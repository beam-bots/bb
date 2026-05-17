# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Unit.Option do
  @moduledoc """
  Functions for specifying and validating units in option schemas.
  """

  alias BB.Unit

  @schema Spark.Options.new!(
            compatible: [
              type: {:or, [:atom, :string]},
              doc: "The provided value must be compatible with this unit",
              required: false
            ],
            min: [
              type: {:custom, __MODULE__, :validate, []},
              doc: "The provided value must be greater than or equal to this value",
              required: false
            ],
            max: [
              type: {:custom, __MODULE__, :validate, []},
              doc: "The provided value must be less than or equal to this value",
              required: false
            ],
            eq: [
              type: {:custom, __MODULE__, :validate, []},
              doc: "The provided value must equal this value",
              required: false
            ]
          )

  @type schema_options :: [schema_option]
  @type schema_option ::
          {:compatible, atom | String.t()}
          | {:min, Localize.Unit.t()}
          | {:max, Localize.Unit.t()}
          | {:eq, Localize.Unit.t()}

  @doc """
  Create a Spark.Options schema type for a unit.

  ## Examples

  Basic usage returns a custom schema type tuple:

      iex> BB.Unit.Option.unit_type()
      {:custom, BB.Unit.Option, :validate, [[]]}

  With compatible option to restrict unit category:

      iex> BB.Unit.Option.unit_type(compatible: :meter)
      {:custom, BB.Unit.Option, :validate, [[compatible: :meter]]}

  With min constraint:

      iex> BB.Unit.Option.unit_type(min: Localize.Unit.new!(0, "meter"))
      {:custom, BB.Unit.Option, :validate, [[min: Localize.Unit.new!(0, "meter")]]}

  With max constraint:

      iex> BB.Unit.Option.unit_type(max: Localize.Unit.new!(100, "meter"))
      {:custom, BB.Unit.Option, :validate, [[max: Localize.Unit.new!(100, "meter")]]}

  Combined constraints:

      iex> BB.Unit.Option.unit_type(compatible: :meter, min: Localize.Unit.new!(0, "meter"), max: Localize.Unit.new!(100, "meter"))
      {:custom, BB.Unit.Option, :validate, [[compatible: :meter, min: Localize.Unit.new!(0, "meter"), max: Localize.Unit.new!(100, "meter")]]}
  """
  @spec unit_type(Keyword.t()) :: {:custom, __MODULE__, :validate, [Keyword.t()]}
  def unit_type(options \\ []) do
    with {:ok, options} <- Spark.Options.validate(options, @schema),
         {:ok, options} <- validate_compatible_option(options),
         {:ok, unit} <- extract_default_unit(options),
         {:ok, options} <- validate_max_option(options, unit),
         {:ok, options} <- validate_min_option(options, unit),
         {:ok, options} <- validate_eq_option(options, unit) do
      # Wrap options in a list so Spark passes them as a single argument
      {:custom, __MODULE__, :validate, [options]}
    else
      {:error, reason} -> raise reason
    end
  end

  @doc """
  Validate a value against a category.

  ## Examples

  Valid unit passes through:

      iex> BB.Unit.Option.validate(Localize.Unit.new!(5, "meter"))
      {:ok, Localize.Unit.new!(5, "meter")}

  Non-unit values are rejected:

      iex> BB.Unit.Option.validate("not a unit")
      {:error, "Value `\\"not a unit\\"` is not a `Localize.Unit` struct"}

  Compatible unit check passes for same category:

      iex> BB.Unit.Option.validate(Localize.Unit.new!(100, "centimeter"), compatible: :meter)
      {:ok, Localize.Unit.new!(100, "centimeter")}

  Incompatible units are rejected:

      iex> BB.Unit.Option.validate(Localize.Unit.new!(90, "degree"), compatible: :meter)
      {:error, "The unit `degree` is not compatible with `meter`"}

  Min constraint - value must be >= min:

      iex> BB.Unit.Option.validate(Localize.Unit.new!(5, "meter"), min: Localize.Unit.new!(1, "meter"))
      {:ok, Localize.Unit.new!(5, "meter")}

  Max constraint - value must be <= max:

      iex> BB.Unit.Option.validate(Localize.Unit.new!(5, "meter"), max: Localize.Unit.new!(10, "meter"))
      {:ok, Localize.Unit.new!(5, "meter")}

  Eq constraint - value must equal exactly:

      iex> BB.Unit.Option.validate(Localize.Unit.new!(5, "meter"), eq: Localize.Unit.new!(5, "meter"))
      {:ok, Localize.Unit.new!(5, "meter")}

  ParamRef values are accepted and annotated with expected unit type:

      iex> ref = BB.Dsl.ParamRef.param([:motion, :max_speed])
      iex> {:ok, validated} = BB.Unit.Option.validate(ref, compatible: :meter)
      iex> validated.expected_unit_type
      :meter
  """
  @spec validate(any, Keyword.t()) ::
          {:ok, Localize.Unit.t()} | {:ok, BB.Dsl.ParamRef.t()} | {:error, String.t()}
  def validate(value, options \\ [])

  def validate(%BB.Dsl.ParamRef{} = ref, options) do
    {:ok, %{ref | expected_unit_type: options[:compatible]}}
  end

  def validate(value, options) do
    with {:ok, value} <- validate_is_unit(value),
         {:ok, value} <- validate_compatible(value, options[:compatible]),
         {:ok, value} <- validate_min(value, options[:min]),
         {:ok, value} <- validate_max(value, options[:max]) do
      validate_eq(value, options[:eq])
    end
  end

  defp validate_is_unit(unit) when is_struct(unit, Localize.Unit), do: {:ok, unit}

  defp validate_is_unit(unit),
    do: {:error, "Value `#{inspect(unit)}` is not a `Localize.Unit` struct"}

  defp validate_compatible(unit, nil), do: {:ok, unit}

  defp validate_compatible(unit, base_unit) do
    if Unit.compatible?(unit, base_unit) do
      {:ok, unit}
    else
      {:error, "The unit `#{unit.name}` is not compatible with `#{base_unit}`"}
    end
  end

  defp validate_min(value, nil), do: {:ok, value}

  defp validate_min(value, min),
    do:
      validate_cmp(
        value,
        min,
        [:gt, :eq],
        "Expected #{Unit.to_string!(value, style: :narrow)} to be greater than or equal to #{Unit.to_string!(min, style: :narrow)}"
      )

  defp validate_max(value, nil), do: {:ok, value}

  defp validate_max(value, max),
    do:
      validate_cmp(
        value,
        max,
        [:lt, :eq],
        "Expected #{Unit.to_string!(value, style: :narrow)} to be less than or equal to #{Unit.to_string!(max, style: :narrow)}"
      )

  defp validate_eq(value, nil), do: {:ok, value}

  defp validate_eq(value, eq),
    do:
      validate_cmp(
        value,
        eq,
        [:eq],
        "Expected #{Unit.to_string!(value, style: :short)} to equal #{Unit.to_string!(eq, style: :short)}"
      )

  defp validate_cmp(value, cmp, valid, message) do
    result = Unit.compare(value, cmp)

    if result in valid do
      {:ok, value}
    else
      {:error, message}
    end
  end

  defp validate_compatible_option(options) do
    if options[:compatible] do
      case Unit.validate_unit(options[:compatible]) do
        {:ok, _} -> {:ok, options}
        {:error, exception} -> {:error, exception}
      end
    else
      {:ok, options}
    end
  end

  defp validate_min_option(options, nil), do: {:ok, options}
  defp validate_min_option(options, type), do: validate_option_by_key(options, :min, type)

  defp validate_max_option(options, nil), do: {:ok, options}
  defp validate_max_option(options, type), do: validate_option_by_key(options, :max, type)

  defp validate_eq_option(options, nil), do: {:ok, options}
  defp validate_eq_option(options, type), do: validate_option_by_key(options, :eq, type)

  defp validate_option_by_key(options, key, type) do
    if options[key] do
      if Unit.compatible?(options[key], type) do
        {:ok, options}
      else
        {:error, "Value for key `#{inspect(key)}` is not compatible with the `#{type}` unit"}
      end
    else
      {:ok, options}
    end
  end

  defp extract_default_unit([]), do: {:ok, nil}

  defp extract_default_unit(options) do
    if options[:compatible] do
      {:ok, options[:compatible]}
    else
      first =
        options
        |> Keyword.take([:min, :max, :eq])
        |> Keyword.values()
        |> List.first()

      {:ok, first}
    end
  end
end
