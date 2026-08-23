# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Type do
  @moduledoc """
  Validation for parameter type definitions in the DSL.

  Parameters can have simple types (`:float`, `:integer`, etc.) or unit types
  like `{:unit, :meter}`.

  Numeric parameters - `:float`, `:integer` and unit types - can also declare
  `min`/`max` bounds. `option_type/3` folds those bounds into the
  `Spark.Options` type generated for the parameter so that they are enforced
  wherever a value is validated, and `describe/1` recovers them from a
  generated type for display.
  """

  alias BB.Unit

  @simple_types [:float, :integer, :boolean, :string, :atom]
  @numeric_types [:float, :integer]

  @type t :: :float | :integer | :boolean | :string | :atom | {:unit, atom}
  @type bound :: number | Localize.Unit.t() | nil

  @doc """
  Validates a parameter type specification.

  Returns `{:ok, type}` for valid types or `{:error, message}` for invalid ones.

  ## Valid Types

  - Simple types: `:float`, `:integer`, `:boolean`, `:string`, `:atom`
  - Unit types: `{:unit, unit_type}` where `unit_type` is a valid CLDR unit

  ## Examples

      iex> BB.Parameter.Type.validate(:float)
      {:ok, :float}

      iex> BB.Parameter.Type.validate({:unit, :meter})
      {:ok, {:unit, :meter}}

      iex> BB.Parameter.Type.validate(:invalid)
      {:error, "Expected one of [:float, :integer, :boolean, :string, :atom] or {:unit, unit_type}, got: :invalid"}
  """
  @spec validate(term()) :: {:ok, t} | {:error, String.t()}
  def validate(type) when type in @simple_types, do: {:ok, type}

  def validate({:unit, unit_type}) when is_atom(unit_type) do
    case Unit.validate_unit(unit_type) do
      {:ok, _} -> {:ok, {:unit, unit_type}}
      {:error, _} -> {:error, "Invalid unit type: #{inspect(unit_type)}"}
    end
  end

  def validate(other) do
    {:error,
     "Expected one of #{inspect(@simple_types)} or {:unit, unit_type}, got: #{inspect(other)}"}
  end

  @doc """
  Builds the `Spark.Options` type for a parameter of `type`, bounded by `min`
  and `max`.

  Either bound may be `nil`. Numeric bounds are plain numbers, bounds on a unit
  type are `Localize.Unit` values compatible with the parameter's unit.

  ## Examples

  An unbounded type is used as-is:

      iex> BB.Parameter.Type.option_type(:float, nil, nil)
      {:ok, :float}

  Bounds wrap the type in a custom validator:

      iex> BB.Parameter.Type.option_type(:integer, 0, 127)
      {:ok, {:custom, BB.Parameter.Type, :validate_bounds, [[type: :integer, min: 0, max: 127]]}}

  Only one of the two is needed:

      iex> BB.Parameter.Type.option_type(:float, 0.0, nil)
      {:ok, {:custom, BB.Parameter.Type, :validate_bounds, [[type: :float, min: 0.0, max: nil]]}}

  Bounds on a unit type become unit constraints:

      iex> BB.Parameter.Type.option_type({:unit, :meter}, nil, Localize.Unit.new!(1, "meter"))
      {:ok, {:custom, BB.Unit.Option, :validate, [[compatible: :meter, max: Localize.Unit.new!(1, "meter")]]}}

  Non-numeric types cannot be bounded:

      iex> BB.Parameter.Type.option_type(:string, 0, nil)
      {:error, "`min` and `max` are only supported for numeric parameter types (:float, :integer or {:unit, unit_type}), got: :string"}
  """
  @spec option_type(t, bound, bound) :: {:ok, Spark.Options.type()} | {:error, String.t()}
  def option_type(type, min, max)

  def option_type({:unit, unit_type}, nil, nil),
    do: {:ok, Unit.Option.unit_type(compatible: unit_type)}

  def option_type(type, nil, nil), do: {:ok, type}

  def option_type({:unit, unit_type}, min, max) do
    with :ok <- validate_unit_bound(:min, min, unit_type),
         :ok <- validate_unit_bound(:max, max, unit_type),
         :ok <- validate_ordering(min, max) do
      bounds = Enum.reject([min: min, max: max], fn {_key, bound} -> is_nil(bound) end)
      {:ok, Unit.Option.unit_type([compatible: unit_type] ++ bounds)}
    end
  end

  def option_type(type, min, max) when type in @numeric_types do
    with :ok <- validate_numeric_bound(:min, min),
         :ok <- validate_numeric_bound(:max, max),
         :ok <- validate_ordering(min, max) do
      {:ok, {:custom, __MODULE__, :validate_bounds, [[type: type, min: min, max: max]]}}
    end
  end

  def option_type(type, _min, _max) do
    {:error,
     "`min` and `max` are only supported for numeric parameter types " <>
       "(:float, :integer or {:unit, unit_type}), got: #{inspect(type)}"}
  end

  @doc """
  Validates a value against a bounded numeric parameter's type and bounds.

  This is the validator used by the type `option_type/3` builds for a bounded
  `:float` or `:integer` parameter.

  ## Examples

      iex> BB.Parameter.Type.validate_bounds(0.5, type: :float, min: 0.0, max: 1.0)
      {:ok, 0.5}

      iex> BB.Parameter.Type.validate_bounds(2.0, type: :float, min: 0.0, max: 1.0)
      {:error, "expected value to be at most 1.0, got: 2.0"}

      iex> BB.Parameter.Type.validate_bounds(-1.0, type: :float, min: 0.0, max: nil)
      {:error, "expected value to be at least 0.0, got: -1.0"}

      iex> BB.Parameter.Type.validate_bounds("nope", type: :float, min: 0.0, max: 1.0)
      {:error, "expected float, got: \\"nope\\""}
  """
  @spec validate_bounds(term, keyword) :: {:ok, number} | {:error, String.t()}
  def validate_bounds(value, bounds) do
    with {:ok, value} <- validate_numeric_type(bounds[:type], value),
         {:ok, value} <- validate_at_least(value, bounds[:min]) do
      validate_at_most(value, bounds[:max])
    end
  end

  @doc """
  Recovers the declared parameter type and its bounds from a generated
  `Spark.Options` type.

  Types which don't carry bounds - including the hand-written schemas of
  components which `use BB.Parameter` - are returned unchanged with `nil`
  bounds.

  ## Examples

      iex> BB.Parameter.Type.describe(:float)
      {:float, nil, nil}

      iex> {:ok, type} = BB.Parameter.Type.option_type(:integer, 0, 127)
      iex> BB.Parameter.Type.describe(type)
      {:integer, 0, 127}

      iex> {:ok, type} = BB.Parameter.Type.option_type({:unit, :meter}, nil, Localize.Unit.new!(1, "meter"))
      iex> BB.Parameter.Type.describe(type)
      {{:unit, :meter}, nil, Localize.Unit.new!(1, "meter")}
  """
  @spec describe(Spark.Options.type() | nil) :: {t | Spark.Options.type() | nil, bound, bound}
  def describe({:custom, __MODULE__, :validate_bounds, [bounds]}),
    do: {bounds[:type], bounds[:min], bounds[:max]}

  def describe({:custom, Unit.Option, :validate, [options]}),
    do: {{:unit, options[:compatible]}, options[:min], options[:max]}

  def describe(type), do: {type, nil, nil}

  defp validate_numeric_type(:float, value) when is_float(value), do: {:ok, value}
  defp validate_numeric_type(:integer, value) when is_integer(value), do: {:ok, value}

  defp validate_numeric_type(type, value),
    do: {:error, "expected #{type}, got: #{inspect(value)}"}

  defp validate_at_least(value, nil), do: {:ok, value}
  defp validate_at_least(value, min) when value >= min, do: {:ok, value}

  defp validate_at_least(value, min),
    do: {:error, "expected value to be at least #{inspect(min)}, got: #{inspect(value)}"}

  defp validate_at_most(value, nil), do: {:ok, value}
  defp validate_at_most(value, max) when value <= max, do: {:ok, value}

  defp validate_at_most(value, max),
    do: {:error, "expected value to be at most #{inspect(max)}, got: #{inspect(value)}"}

  defp validate_numeric_bound(_key, bound) when is_number(bound) or is_nil(bound), do: :ok

  defp validate_numeric_bound(key, bound),
    do: {:error, "`#{key}` must be a number, got: #{inspect(bound)}"}

  defp validate_unit_bound(_key, nil, _unit_type), do: :ok

  defp validate_unit_bound(key, bound, unit_type) when is_struct(bound, Localize.Unit) do
    if Unit.compatible?(bound, unit_type) do
      :ok
    else
      {:error, "`#{key}` must be compatible with `#{unit_type}`, got: #{Unit.describe(bound)}"}
    end
  end

  defp validate_unit_bound(key, bound, unit_type) do
    {:error,
     "`#{key}` must be a `Localize.Unit` compatible with `#{unit_type}`, got: #{inspect(bound)}"}
  end

  defp validate_ordering(nil, _max), do: :ok
  defp validate_ordering(_min, nil), do: :ok

  defp validate_ordering(min, max) when is_number(min) and is_number(max) do
    if min <= max do
      :ok
    else
      {:error, "`min` must not be greater than `max`, got: #{inspect(min)} and #{inspect(max)}"}
    end
  end

  defp validate_ordering(min, max) do
    if Unit.compare(min, max) in [:lt, :eq] do
      :ok
    else
      {:error,
       "`min` must not be greater than `max`, got: #{Unit.describe(min)} and #{Unit.describe(max)}"}
    end
  end
end
