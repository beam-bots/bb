# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Unit do
  @moduledoc """
  A physical quantity — a magnitude and the unit it is measured in.

  Conversion is backed by `BB.Unit.Conversions`, a module generated from CLDR
  data by [Localize](https://hexdocs.pm/localize) and committed into this
  repository. Localize itself is a build-time dependency only, so a robot
  running on a Nerves target carries the conversion tables rather than the
  whole localisation library.

  Unit identifiers can be passed as either atoms (`:newton_meter`) or strings
  (`"newton-meter"`); the helpers in this module convert atoms with
  underscores into the CLDR canonical dash form before passing them through.
  """

  alias BB.Error.Invalid
  alias BB.Unit.Conversions

  defstruct [:name, :value]

  @type t :: %__MODULE__{name: String.t(), value: number}

  @doc """
  Compare two units of the same dimensional category.

  Both are reduced to their common base unit before comparing, so units of
  differing scale compare correctly.

  ## Examples

      iex> import BB.Unit
      iex> BB.Unit.compare(~u(100 centimeter), ~u(1 meter))
      :eq

      iex> import BB.Unit
      iex> BB.Unit.compare(~u(90 degree), ~u(1 radian))
      :gt

  Incompatible units return an error:

      iex> import BB.Unit
      iex> {:error, error} = BB.Unit.compare(~u(1 meter), ~u(1 second))
      iex> Exception.message(error)
      "The unit `meter` is not compatible with `second`"
  """
  @spec compare(t, t) :: :lt | :eq | :gt | {:error, Exception.t()}
  def compare(%__MODULE__{} = a, %__MODULE__{} = b) do
    with {:ok, base_a, value_a} <- to_base(a),
         {:ok, base_b, value_b} <- to_base(b) do
      cond do
        base_a != base_b -> {:error, incompatible(a.name, b.name)}
        value_a < value_b -> :lt
        value_a > value_b -> :gt
        true -> :eq
      end
    end
  end

  @doc """
  Check whether two units belong to the same dimensional category.

  Either argument may be a `t:t/0`, an atom or a string.

  ## Examples

      iex> BB.Unit.compatible?(:centimeter, :meter)
      true

      iex> BB.Unit.compatible?(:degree, :meter)
      false
  """
  @spec compatible?(t | atom | binary, t | atom | binary) :: boolean
  def compatible?(a, b) do
    case {base_unit(a), base_unit(b)} do
      {{:ok, base}, {:ok, base}} -> true
      _mismatch -> false
    end
  end

  @doc """
  Convert a unit to another unit of the same dimensional category.

  ## Examples

      iex> import BB.Unit
      iex> {:ok, converted} = BB.Unit.convert(~u(100 centimeter), :meter)
      iex> {converted.name, converted.value}
      {"meter", 1.0}

      iex> import BB.Unit
      iex> {:error, error} = BB.Unit.convert(~u(1 meter), :second)
      iex> Exception.message(error)
      "The unit `meter` is not compatible with `second`"
  """
  @spec convert(t, atom | binary) :: {:ok, t} | {:error, Exception.t()}
  def convert(%__MODULE__{} = unit, target) do
    target = unit_name(target)

    with {:ok, base, value} <- to_base(unit),
         {:ok, {^base, factor, offset}} <- resolve(target) do
      {:ok, %__MODULE__{name: target, value: (value - offset) / factor}}
    else
      {:ok, {_other_base, _factor, _offset}} -> {:error, incompatible(unit.name, target)}
      {:error, error} -> {:error, error}
    end
  end

  @doc """
  Convert a unit to another unit of the same dimensional category, or raise.
  """
  @spec convert!(t, atom | binary) :: t | no_return
  def convert!(%__MODULE__{} = unit, target) do
    case convert(unit, target) do
      {:ok, converted} -> converted
      {:error, error} -> raise error
    end
  end

  @doc """
  Build a unit from a magnitude and an identifier.

  ## Examples

      iex> {:ok, unit} = BB.Unit.new(5, :meter)
      iex> {unit.name, unit.value}
      {"meter", 5}

      iex> {:error, error} = BB.Unit.new(5, :bogus)
      iex> Exception.message(error)
      "`bogus` is not a known unit"
  """
  @spec new(number, atom | binary) :: {:ok, t} | {:error, Exception.t()}
  def new(value, name) when is_number(value) do
    name = unit_name(name)

    with {:ok, _resolved} <- resolve(name) do
      {:ok, %__MODULE__{name: name, value: value}}
    end
  end

  @doc """
  Build a unit from a magnitude and an identifier, or raise.
  """
  @spec new!(number, atom | binary) :: t | no_return
  def new!(value, name) do
    case new(value, name) do
      {:ok, unit} -> unit
      {:error, error} -> raise error
    end
  end

  @doc """
  Parse a string input as a unit.

  The input should be a magnitude (integer or float) followed by a unit name.
  Whitespace between the magnitude and unit is optional.

  Units are generally referred to in the singular, even if it doesn't read as
  nicely, for example `meter_per_second` rather than `meters_per_second`.

  ## Examples

  Integer magnitudes:

      iex> import BB.Unit
      iex> u = ~u(5 meter)
      iex> {u.name, u.value}
      {"meter", 5}

  Float magnitudes:

      iex> import BB.Unit
      iex> u = ~u(0.1 meter)
      iex> {u.name, u.value}
      {"meter", 0.1}

  Negative values:

      iex> import BB.Unit
      iex> u = ~u(-90 degree)
      iex> {u.name, u.value}
      {"degree", -90}

  Whitespace is optional:

      iex> import BB.Unit
      iex> u = ~u(100centimeter)
      iex> {u.name, u.value}
      {"centimeter", 100}

  Compound units use underscores, which are translated to the CLDR dash form:

      iex> import BB.Unit
      iex> u = ~u(10 meter_per_second)
      iex> {u.name, u.value}
      {"meter-per-second", 10}
  """
  @spec sigil_u(binary, charlist) :: t | no_return
  def sigil_u(input, []) do
    with :error <- maybe_parse_as_integer(input),
         :error <- maybe_parse_as_float(input) do
      raise "Invalid input `#{inspect(input)}`"
    else
      {:ok, magnitude, unit} -> new!(magnitude, unit)
    end
  end

  @doc """
  Render a unit as its magnitude followed by its CLDR identifier.

      iex> import BB.Unit
      iex> BB.Unit.to_string!(~u(1.5 meter_per_second))
      "1.5 meter-per-second"
  """
  @spec to_string!(t) :: String.t()
  def to_string!(%__MODULE__{name: name, value: value}), do: "#{value} #{name}"

  @doc """
  Convert an atom or underscored string unit identifier to the CLDR canonical
  dash form.

      iex> BB.Unit.unit_name(:newton_meter)
      "newton-meter"

      iex> BB.Unit.unit_name("meter_per_second")
      "meter-per-second"

      iex> BB.Unit.unit_name("meter")
      "meter"
  """
  @spec unit_name(atom | binary) :: binary
  def unit_name(name) when is_atom(name), do: name |> Atom.to_string() |> unit_name()
  def unit_name(name) when is_binary(name), do: String.replace(name, "_", "-")

  @doc """
  Validate that an identifier resolves to a known unit.

  Returns `{:ok, canonical_name}` for a known unit, `{:error, exception}`
  otherwise.

      iex> BB.Unit.validate_unit(:newton_meter)
      {:ok, "newton-meter"}
  """
  @spec validate_unit(atom | binary) :: {:ok, binary} | {:error, Exception.t()}
  def validate_unit(name) do
    name = unit_name(name)

    with {:ok, _resolved} <- resolve(name) do
      {:ok, name}
    end
  end

  defp base_unit(%__MODULE__{name: name}), do: base_unit(name)

  defp base_unit(name) do
    with {:ok, {base, _factor, _offset}} <- resolve(unit_name(name)) do
      {:ok, base}
    end
  end

  defp incompatible(name, expected), do: Invalid.Unit.exception(unit: name, expected: expected)

  defp maybe_parse_as_float(input) do
    case Float.parse(input) do
      {magnitude, unit} -> {:ok, magnitude, String.trim(unit)}
      :error -> :error
    end
  end

  defp maybe_parse_as_integer(input) do
    case Integer.parse(input) do
      {_, "." <> _} -> :error
      {magnitude, unit} -> {:ok, magnitude, String.trim(unit)}
      :error -> :error
    end
  end

  defp resolve(name) do
    case Conversions.resolve(name) do
      {:ok, resolved} -> {:ok, resolved}
      {:error, {:unknown_unit, unknown}} -> {:error, Invalid.Unit.exception(unit: unknown)}
    end
  end

  defp to_base(%__MODULE__{name: name, value: value}) do
    with {:ok, {base, factor, offset}} <- resolve(name) do
      {:ok, base, value * factor + offset}
    end
  end
end

defimpl Inspect, for: BB.Unit do
  def inspect(%BB.Unit{name: name, value: value}, _opts) do
    "BB.Unit.new!(#{Kernel.inspect(value)}, #{Kernel.inspect(name)})"
  end
end
