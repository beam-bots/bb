# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Component.OptionsSchema do
  @moduledoc false

  # Shared `options_schema/0` machinery for the component behaviours
  # (`BB.Sensor`, `BB.Actuator`, `BB.Controller`, `BB.Estimator`,
  # `BB.Bridge`, `BB.Command`).
  #
  # A component declares its schema one of two ways:
  #
  #   * `use BB.Sensor, options_schema: [...]` — the keyword form, for
  #     self-contained literal schemas.
  #   * `def options_schema, do: ...` — the callback form, for schemas that
  #     reference module attributes, helpers, or `~u` sigils (which aren't in
  #     scope where `use` expands).
  #
  # Providing both is a compile error. Providing neither yields an empty
  # schema, so `options_schema/0` is always defined and callers never need to
  # guard with `function_exported?/3`.

  @doc """
  Validate the user-facing options against `module.options_schema/0`,
  applying schema defaults, and merge the framework-injected keys back in.

  `framework_keys` are the keys a component server injects (e.g. `:bb`,
  `:sensor_profile`) which are not part of the user schema; they are split
  off before validation and merged back onto the validated result.

  Assumes `module` implements the relevant component behaviour, so
  `options_schema/0` is defined (the `use BB.X` macros guarantee it).
  Behaviour conformance itself is enforced at compile time by
  `BB.Dsl.Verifiers.ValidateChildSpecs`.
  """
  @spec validate(module(), keyword(), [atom()]) ::
          {:ok, keyword()} | {:error, Spark.Options.ValidationError.t()}
  def validate(module, resolved_opts, framework_keys) do
    {framework_opts, user_opts} = Keyword.split(resolved_opts, framework_keys)

    case Spark.Options.validate(user_opts, module.options_schema()) do
      {:ok, validated} -> {:ok, Keyword.merge(framework_opts, validated)}
      {:error, _} = error -> error
    end
  end

  @doc false
  def inject(behaviour, schema_opts) do
    given? = not is_nil(schema_opts)

    schema_ast =
      if given? do
        quote do: Spark.Options.new!(unquote(schema_opts))
      end

    quote do
      @__bb_options_schema_behaviour unquote(behaviour)
      @__bb_options_schema_given unquote(given?)
      @__bb_options_schema unquote(schema_ast)
      @before_compile BB.Component.OptionsSchema
    end
  end

  @doc false
  defmacro __before_compile__(env) do
    behaviour = Module.get_attribute(env.module, :__bb_options_schema_behaviour)
    given? = Module.get_attribute(env.module, :__bb_options_schema_given)
    user_defined? = Module.defines?(env.module, {:options_schema, 0})

    cond do
      given? and user_defined? ->
        raise CompileError,
          file: env.file,
          line: env.line,
          description:
            "#{inspect(env.module)} passes `options_schema:` to " <>
              "`use #{inspect(behaviour)}` and also defines `options_schema/0`. " <>
              "Declare the schema one way, not both."

      given? ->
        quote do
          @impl unquote(behaviour)
          def options_schema, do: @__bb_options_schema
        end

      user_defined? ->
        []

      true ->
        quote do
          @impl unquote(behaviour)
          def options_schema, do: Spark.Options.new!([])
        end
    end
  end
end
