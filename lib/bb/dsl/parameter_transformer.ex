# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ParameterTransformer do
  @moduledoc """
  Generates parameter schema and default values from DSL definitions.

  This transformer processes the `parameters` section, generating:
  - `__bb_parameter_schema__/0` - Returns the Spark.Options schema for validation
  - `__bb_default_parameters__/0` - Returns default values as `{path, value}` tuples

  At runtime, these are used by `BB.Robot.Runtime` to register parameters with
  proper validation schemas.

  A parameter's `min`/`max` bounds are folded into its generated type by
  `BB.Parameter.Type.option_type/3`; bounds which can't be enforced - or a
  default which doesn't satisfy them - fail the robot's compilation.
  """
  use Spark.Dsl.Transformer
  alias BB.Dsl.{Param, ParamGroup}
  alias BB.Parameter.Type
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @doc false
  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(BB.Dsl.RobotTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    with {:ok, schema_opts, defaults} <-
           dsl |> Transformer.get_entities([:parameters]) |> collect_parameters([], module) do
      if Enum.empty?(schema_opts) do
        inject_empty_functions(dsl)
      else
        inject_parameter_functions(dsl, schema_opts, defaults)
      end
    end
  end

  defp collect_parameters(entities, path_prefix, module) do
    Enum.reduce_while(entities, {:ok, [], []}, fn entity, {:ok, schema_acc, defaults_acc} ->
      case collect_entity(entity, path_prefix, module) do
        {:ok, schema, defaults} ->
          {:cont, {:ok, schema_acc ++ schema, defaults_acc ++ defaults}}

        {:error, error} ->
          {:halt, {:error, error}}
      end
    end)
  end

  defp collect_entity(%Param{} = param, path_prefix, module) do
    path = path_prefix ++ [param.name]

    with {:ok, schema_opts} <- build_schema_opts(param, path, module) do
      {:ok, [{path, schema_opts}], defaults_for(param, path)}
    end
  end

  defp collect_entity(%ParamGroup{} = group, path_prefix, module) do
    collect_parameters(group.params ++ group.groups, path_prefix ++ [group.name], module)
  end

  defp collect_entity(_entity, _path_prefix, _module), do: {:ok, [], []}

  defp defaults_for(%Param{default: nil}, _path), do: []
  defp defaults_for(%Param{default: default}, path), do: [{path, default}]

  defp build_schema_opts(%Param{} = param, path, module) do
    with {:ok, type} <- option_type(param, path, module),
         :ok <- validate_default(param, type, path, module) do
      opts = [type: type]

      opts = if param.doc, do: Keyword.put(opts, :doc, param.doc), else: opts
      opts = if param.default != nil, do: Keyword.put(opts, :default, param.default), else: opts

      {:ok, opts}
    end
  end

  defp option_type(%Param{} = param, path, module) do
    case Type.option_type(param.type, param.min, param.max) do
      {:ok, type} -> {:ok, type}
      {:error, message} -> {:error, dsl_error(path, module, message)}
    end
  end

  defp validate_default(%Param{default: nil}, _type, _path, _module), do: :ok
  defp validate_default(%Param{min: nil, max: nil}, _type, _path, _module), do: :ok

  # Bounding a parameter always produces a custom validator, so the declared
  # default can be run through it directly.
  defp validate_default(%Param{default: default}, {:custom, mod, fun, args}, path, module) do
    case apply(mod, fun, [default | args]) do
      {:ok, _default} -> :ok
      {:error, message} -> {:error, dsl_error(path, module, "invalid `default`: #{message}")}
    end
  end

  defp dsl_error(path, module, message) do
    DslError.exception(module: module, path: [:parameters | path], message: message)
  end

  defp inject_empty_functions(dsl) do
    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @doc false
         @spec __bb_parameter_schema__() :: [{[atom()], keyword()}]
         def __bb_parameter_schema__, do: []

         @doc false
         @spec __bb_default_parameters__() :: [{[atom()], term()}]
         def __bb_default_parameters__, do: []
       end
     )}
  end

  defp inject_parameter_functions(dsl, schema_opts, defaults) do
    schema_data = Macro.escape(schema_opts)
    defaults_data = Macro.escape(defaults)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @bb_parameter_schema unquote(schema_data)
         @bb_default_parameters unquote(defaults_data)

         @doc """
         Returns the parameter schema for validation.

         The schema is a list of `{path, opts}` tuples where `opts` is a keyword
         list compatible with `Spark.Options`.
         """
         @spec __bb_parameter_schema__() :: [{[atom()], keyword()}]
         def __bb_parameter_schema__, do: @bb_parameter_schema

         @doc """
         Returns default parameter values.

         Returns a list of `{path, value}` tuples for parameters that have defaults.
         """
         @spec __bb_default_parameters__() :: [{[atom()], term()}]
         def __bb_default_parameters__, do: @bb_default_parameters
       end
     )}
  end
end
