# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Unit do
  @moduledoc """
  Helpers for working with units in BB DSLs.

  Wraps `Localize.Unit` and provides a sigil for compact unit literals.
  Unit identifiers can be passed as either atoms (`:newton_meter`) or strings
  (`"newton-meter"`); the helpers in this module convert atoms with
  underscores into the CLDR canonical dash form before passing them through.
  """

  @doc """
  Parse a string input as a unit.

  The input should be a magnitude (integer or float) followed by a unit name.
  Whitespace between the magnitude and unit is optional.

  Units are generally referred to in the singular, even if it doesn't read as
  nicely, for example `meter_per_second` rather than `meters_per_second`. For
  a full list of supported units, see the
  [Localize documentation](https://hexdocs.pm/localize/units.html).

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
  @spec sigil_u(binary, charlist) :: Localize.Unit.t() | no_return
  def sigil_u(input, []) do
    with :error <- maybe_parse_as_integer(input),
         :error <- maybe_parse_as_float(input) do
      raise "Invalid input `#{inspect(input)}`"
    else
      {:ok, magnitude, unit} -> Localize.Unit.new!(magnitude, unit_name(unit))
    end
  end

  @doc """
  Convert an atom or underscored string unit identifier to the CLDR canonical
  dash form that `Localize.Unit` expects.

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

  Returns `{:ok, unit}` for a known unit, `{:error, exception}` otherwise.
  """
  @spec validate_unit(atom | binary) :: {:ok, Localize.Unit.t()} | {:error, Exception.t()}
  def validate_unit(name), do: name |> unit_name() |> Localize.Unit.new()

  @doc """
  Check whether two units belong to the same dimensional category.

  Accepts atoms or strings as the second argument and translates them to the
  CLDR canonical form. Either argument may also be a `Localize.Unit` struct.
  """
  @spec compatible?(Localize.Unit.t() | atom | binary, Localize.Unit.t() | atom | binary) ::
          boolean
  def compatible?(a, b) when is_atom(a), do: compatible?(unit_name(a), b)
  def compatible?(a, b) when is_atom(b), do: compatible?(a, unit_name(b))
  def compatible?(a, b), do: Localize.Unit.compatible?(a, b)

  @doc "Delegates to `Localize.Unit.compare/2`."
  @spec compare(Localize.Unit.t(), Localize.Unit.t()) ::
          :lt | :eq | :gt | {:error, Exception.t()}
  defdelegate compare(a, b), to: Localize.Unit

  @doc "Delegates to `Localize.Unit.to_string!/2`."
  @spec to_string!(Localize.Unit.t() | [Localize.Unit.t()], keyword) :: String.t()
  defdelegate to_string!(unit, options \\ []), to: Localize.Unit

  defimpl Inspect, for: Localize.Unit do
    def inspect(%Localize.Unit{name: name, value: value}, _opts) do
      "Localize.Unit.new!(#{Kernel.inspect(value)}, #{Kernel.inspect(name)})"
    end
  end

  defp maybe_parse_as_integer(input) do
    case Integer.parse(input) do
      {_, "." <> _} -> :error
      {magnitude, unit} -> {:ok, magnitude, String.trim(unit)}
      :error -> :error
    end
  end

  defp maybe_parse_as_float(input) do
    case Float.parse(input) do
      {magnitude, unit} -> {:ok, magnitude, String.trim(unit)}
      :error -> :error
    end
  end
end
