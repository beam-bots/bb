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
  """
  use Spark.Dsl.Transformer
  alias BB.Dsl.{Param, ParamGroup}
  alias BB.Unit
  alias Spark.Dsl.Transformer

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
    {schema_opts, defaults} =
      dsl
      |> Transformer.get_entities([:parameters])
      |> collect_parameters([])

    if Enum.empty?(schema_opts) do
      inject_empty_functions(dsl)
    else
      inject_parameter_functions(dsl, schema_opts, defaults)
    end
  end

  defp collect_parameters(entities, path_prefix) do
    Enum.reduce(entities, {[], []}, fn entity, {schema_acc, defaults_acc} ->
      case entity do
        %Param{} = param ->
          {param_schema, param_defaults} = process_param(param, path_prefix)
          {schema_acc ++ param_schema, defaults_acc ++ param_defaults}

        %ParamGroup{} = group ->
          group_path = path_prefix ++ [group.name]

          {nested_schema, nested_defaults} =
            collect_parameters(group.params ++ group.groups, group_path)

          {schema_acc ++ nested_schema, defaults_acc ++ nested_defaults}

        _ ->
          {schema_acc, defaults_acc}
      end
    end)
  end

  defp process_param(%Param{} = param, path_prefix) do
    path = path_prefix ++ [param.name]
    schema_opts = build_schema_opts(param)

    defaults =
      if param.default != nil do
        [{path, param.default}]
      else
        []
      end

    {[{path, schema_opts}], defaults}
  end

  defp build_schema_opts(%Param{} = param) do
    opts = [type: convert_param_type(param.type)]

    opts = if param.doc, do: Keyword.put(opts, :doc, param.doc), else: opts
    opts = if param.default != nil, do: Keyword.put(opts, :default, param.default), else: opts

    opts
  end

  defp convert_param_type({:unit, unit_type}) do
    Unit.Option.unit_type(compatible: unit_type)
  end

  defp convert_param_type(type), do: type

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
