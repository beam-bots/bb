defmodule Kinetix.Unit.Option do
  @moduledoc """
  Functions for specifying and validating units in option schemas.
  """

  alias Kinetix.Cldr.Unit

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
          | {:min, Cldr.Unit.t()}
          | {:max, Cldr.Unit.t()}
          | {:eq, Cldr.Unit.t()}

  @doc """
  Create a Spark.Options schema type for a unit.

  ## Examples

  Basic usage returns a custom schema type tuple:

      iex> Kinetix.Unit.Option.unit_type()
      {:custom, Kinetix.Unit.Option, :validate, [[]]}

  With compatible option to restrict unit category:

      iex> Kinetix.Unit.Option.unit_type(compatible: :meter)
      {:custom, Kinetix.Unit.Option, :validate, [[compatible: :meter]]}

  With min constraint:

      iex> Kinetix.Unit.Option.unit_type(min: Cldr.Unit.new!(:meter, 0))
      {:custom, Kinetix.Unit.Option, :validate, [[min: Cldr.Unit.new!(:meter, 0)]]}

  With max constraint:

      iex> Kinetix.Unit.Option.unit_type(max: Cldr.Unit.new!(:meter, 100))
      {:custom, Kinetix.Unit.Option, :validate, [[max: Cldr.Unit.new!(:meter, 100)]]}

  Combined constraints:

      iex> Kinetix.Unit.Option.unit_type(compatible: :meter, min: Cldr.Unit.new!(:meter, 0), max: Cldr.Unit.new!(:meter, 100))
      {:custom, Kinetix.Unit.Option, :validate, [[compatible: :meter, min: Cldr.Unit.new!(:meter, 0), max: Cldr.Unit.new!(:meter, 100)]]}
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

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 5))
      {:ok, Cldr.Unit.new!(:meter, 5)}

  Non-unit values are rejected:

      iex> Kinetix.Unit.Option.validate("not a unit")
      {:error, "Value `\\"not a unit\\"` is not a `Cldr.Unit` struct"}

  Compatible unit check passes for same category:

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:centimeter, 100), compatible: :meter)
      {:ok, Cldr.Unit.new!(:centimeter, 100)}

  Incompatible units are rejected:

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:degree, 90), compatible: :meter)
      {:error, "The unit `degree` is not compatible with `meter`"}

  Min constraint - value must be >= min:

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 5), min: Cldr.Unit.new!(:meter, 1))
      {:ok, Cldr.Unit.new!(:meter, 5)}

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 1), min: Cldr.Unit.new!(:meter, 5))
      {:error, "Expected 1m to be greater than or equal to 5m"}

  Max constraint - value must be <= max:

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 5), max: Cldr.Unit.new!(:meter, 10))
      {:ok, Cldr.Unit.new!(:meter, 5)}

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 15), max: Cldr.Unit.new!(:meter, 10))
      {:error, "Expected 15m to be less than or equal to 10m"}

  Eq constraint - value must equal exactly:

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 5), eq: Cldr.Unit.new!(:meter, 5))
      {:ok, Cldr.Unit.new!(:meter, 5)}

      iex> Kinetix.Unit.Option.validate(Cldr.Unit.new!(:meter, 5), eq: Cldr.Unit.new!(:meter, 10))
      {:error, "Expected 5 m to equal 10 m"}
  """
  @spec validate(any, Keyword.t()) :: {:ok, Cldr.Unit.t()} | {:error, String.t()}
  def validate(value, options \\ []) do
    with {:ok, value} <- validate_is_unit(value),
         {:ok, value} <- validate_compatible(value, options[:compatible]),
         {:ok, value} <- validate_min(value, options[:min]),
         {:ok, value} <- validate_max(value, options[:max]) do
      validate_eq(value, options[:eq])
    end
  end

  defp validate_is_unit(unit) when is_struct(unit, Cldr.Unit), do: {:ok, unit}

  defp validate_is_unit(unit),
    do: {:error, "Value `#{inspect(unit)}` is not a `Cldr.Unit` struct"}

  defp validate_compatible(unit, nil), do: {:ok, unit}

  defp validate_compatible(unit, base_unit) do
    if Unit.compatible?(unit, base_unit) do
      {:ok, unit}
    else
      {:error, "The unit `#{unit.unit}` is not compatible with `#{base_unit}`"}
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
    result = Cldr.Unit.compare(value, cmp)

    if result in valid do
      {:ok, value}
    else
      {:error, message}
    end
  end

  defp validate_compatible_option(options) do
    if options[:compatible] do
      case Unit.validate_unit(options[:compatible]) do
        {:ok, _, _} -> {:ok, options}
        {:error, {module, message}} -> {:error, module.exception(message: message)}
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
    if options[:comptable] do
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
