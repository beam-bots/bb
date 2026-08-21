# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ChildSpecOptions do
  @moduledoc """
  A child spec's options, resolved as far as compile time allows.

  Used by the verifiers that need to ask a component module about itself before
  the robot exists - what options it accepts, what its driver can do - so they
  all take the same view of a `param()` reference.

  Which is that they can't see one. A parameter isn't resolved until the robot
  is running and its store has been populated, so a parameterised option is
  dropped here and its key is not required, leaving the schema's default in its
  place. Anything reading these options has to answer for the default rather
  than for the value the robot will actually run with.
  """

  alias BB.Dsl.ParamRef

  @doc """
  Split a child spec into its module and options.
  """
  @spec module_and_options(module() | {module(), keyword()}) :: {module(), keyword()}
  def module_and_options(module) when is_atom(module), do: {module, []}
  def module_and_options({module, opts}) when is_atom(module), do: {module, opts}

  @doc """
  Validate options against the module's `options_schema/0`, applying defaults.

  Parameterised options are dropped and their keys made optional, so a spec
  that is only valid once its parameters resolve still validates here.
  """
  @spec validate(module(), keyword()) ::
          {:ok, keyword()} | {:error, Spark.Options.ValidationError.t()}
  def validate(module, opts) do
    schema = mark_optional(module.options_schema(), param_ref_keys(opts))

    opts
    |> Enum.reject(&param_ref?/1)
    |> Spark.Options.validate(schema)
  end

  defp param_ref_keys(opts) do
    opts
    |> Enum.filter(&param_ref?/1)
    |> Keyword.keys()
  end

  defp param_ref?({_key, value}), do: is_struct(value, ParamRef)

  defp mark_optional(%Spark.Options{schema: schema} = spark_opts, keys) do
    %{spark_opts | schema: mark_optional(schema, keys)}
  end

  defp mark_optional(schema, keys) when is_list(schema) do
    Enum.map(schema, fn {key, opts} ->
      if key in keys do
        {key, Keyword.put(opts, :required, false)}
      else
        {key, opts}
      end
    end)
  end
end
