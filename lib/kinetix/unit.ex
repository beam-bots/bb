defmodule Kinetix.Unit do
  @moduledoc """
  Helpers for working with units in Kinetix DSLs.
  """

  @doc """
  Parse a string input as a unit.

  The input should be a magnitude (integer or float) followed by a unit name.
  Whitespace between the magnitude and unit is optional.

  For a full list of supported units, see the
  [ex_cldr_units documentation](https://hexdocs.pm/ex_cldr_units/readme.html).

  ## Examples

  Integer magnitudes:

      iex> import Kinetix.Unit
      iex> ~u(5 meter)
      Cldr.Unit.new!(:meter, 5)

  Float magnitudes:

      iex> import Kinetix.Unit
      iex> ~u(0.1 meter)
      Cldr.Unit.new!(Decimal.new("0.1"), :meter)

  Negative values:

      iex> import Kinetix.Unit
      iex> ~u(-90 degree)
      Cldr.Unit.new!(:degree, -90)

  Whitespace is optional:

      iex> import Kinetix.Unit
      iex> ~u(100centimeter)
      Cldr.Unit.new!(:centimeter, 100)

  Compound units:

      iex> import Kinetix.Unit
      iex> ~u(10 meter_per_second)
      Cldr.Unit.new!(:meter_per_second, 10)
  """
  @spec sigil_u(binary, charlist) :: Cldr.Unit.t() | no_return
  def sigil_u(input, []) do
    with :error <- maybe_parse_as_integer(input),
         :error <- maybe_parse_as_float(input) do
      raise "Invalid input `#{inspect(input)}`"
    else
      {:ok, magnitude, unit} -> Cldr.Unit.new!(magnitude, unit)
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
