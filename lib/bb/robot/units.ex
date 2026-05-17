# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Units do
  @moduledoc """
  Unit conversion functions for transforming `Localize.Unit` values into base
  SI floats.

  All functions in this module convert from `Localize.Unit.t()` structs to
  native floats in SI base units, suitable for efficient numerical
  computation.
  """

  @doc """
  Convert a length unit to meters (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_meters(~u(100 centimeter))
      1.0

      iex> import BB.Unit
      iex> BB.Robot.Units.to_meters(~u(1.5 meter))
      1.5
  """
  @spec to_meters(Localize.Unit.t()) :: float()
  def to_meters(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("meter")
    |> extract_float()
  end

  @doc """
  Convert an angle unit to radians (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_radians(~u(180 degree))
      :math.pi()

      iex> import BB.Unit
      iex> BB.Robot.Units.to_radians(~u(0 degree))
      0.0
  """
  @spec to_radians(Localize.Unit.t()) :: float()
  def to_radians(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("radian")
    |> extract_float()
  end

  @doc """
  Convert a mass unit to kilograms (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_kilograms(~u(1000 gram))
      1.0

      iex> import BB.Unit
      iex> BB.Robot.Units.to_kilograms(~u(2.5 kilogram))
      2.5
  """
  @spec to_kilograms(Localize.Unit.t()) :: float()
  def to_kilograms(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("kilogram")
    |> extract_float()
  end

  @doc """
  Convert a moment of inertia unit to kg·m² (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_kilogram_square_meters(~u(0.5 kilogram_square_meter))
      0.5
  """
  @spec to_kilogram_square_meters(Localize.Unit.t()) :: float()
  def to_kilogram_square_meters(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("kilogram-square-meter")
    |> extract_float()
  end

  @doc """
  Convert a force unit to newtons (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_newtons(~u(10 newton))
      10.0
  """
  @spec to_newtons(Localize.Unit.t()) :: float()
  def to_newtons(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("newton")
    |> extract_float()
  end

  @doc """
  Convert a force unit to newtons (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_newton(~u(5 newton))
      5.0
  """
  @spec to_newton(Localize.Unit.t()) :: float()
  def to_newton(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("newton")
    |> extract_float()
  end

  @doc """
  Convert a torque unit to newton-meters (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_newton_meters(~u(5 newton_meter))
      5.0
  """
  @spec to_newton_meters(Localize.Unit.t()) :: float()
  def to_newton_meters(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("newton-meter")
    |> extract_float()
  end

  @doc """
  Convert a linear velocity unit to meters per second (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_meters_per_second(~u(10 meter_per_second))
      10.0
  """
  @spec to_meters_per_second(Localize.Unit.t()) :: float()
  def to_meters_per_second(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("meter-per-second")
    |> extract_float()
  end

  @doc """
  Convert an angular velocity unit to radians per second (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_radians_per_second(~u(180 degree_per_second))
      :math.pi()
  """
  @spec to_radians_per_second(Localize.Unit.t()) :: float()
  def to_radians_per_second(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("radian-per-second")
    |> extract_float()
  end

  @doc """
  Convert a linear acceleration unit to metres per second squared (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_meters_per_square_second(~u(9.81 meter_per_square_second))
      9.81
  """
  @spec to_meters_per_square_second(Localize.Unit.t()) :: float()
  def to_meters_per_square_second(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("meter-per-square-second")
    |> extract_float()
  end

  @doc """
  Convert an angular acceleration unit to radians per second squared (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_radians_per_square_second(~u(360 degree_per_square_second))
      :math.pi() * 2
  """
  @spec to_radians_per_square_second(Localize.Unit.t()) :: float()
  def to_radians_per_square_second(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("radian-per-square-second")
    |> extract_float()
  end

  @doc """
  Convert a linear damping coefficient to N·s/m (float).

  ## Examples

      iex> import BB.Unit
      iex> BB.Robot.Units.to_linear_damping(~u(1.5 newton_second_per_meter))
      1.5
  """
  @spec to_linear_damping(Localize.Unit.t()) :: float()
  def to_linear_damping(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("newton-second-per-meter")
    |> extract_float()
  end

  @doc """
  Convert a rotational damping coefficient to N·m·s/rad (float).

  Note: The DSL uses `newton_meter_second_per_degree` but we convert
  to radians for consistency with other angular quantities.
  """
  @spec to_rotational_damping(Localize.Unit.t()) :: float()
  def to_rotational_damping(%Localize.Unit{} = unit) do
    unit
    |> Localize.Unit.convert!("newton-meter-second-per-radian")
    |> extract_float()
  end

  @doc """
  Extract the numeric value from a `Localize.Unit` as a float.

  Handles integer, float, and `Decimal` values.
  """
  @spec extract_float(Localize.Unit.t()) :: float()
  def extract_float(%Localize.Unit{value: value}) when is_integer(value) do
    value / 1
  end

  def extract_float(%Localize.Unit{value: value}) when is_float(value) do
    value
  end

  def extract_float(%Localize.Unit{value: %Decimal{} = value}) do
    Decimal.to_float(value)
  end

  @doc """
  Convert an optional unit value to its base SI float, or return nil.
  """
  @spec to_meters_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_meters_or_nil(nil), do: nil
  def to_meters_or_nil(unit), do: to_meters(unit)

  @spec to_radians_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_radians_or_nil(nil), do: nil
  def to_radians_or_nil(unit), do: to_radians(unit)

  @spec to_kilograms_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_kilograms_or_nil(nil), do: nil
  def to_kilograms_or_nil(unit), do: to_kilograms(unit)

  @spec to_kilogram_square_meters_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_kilogram_square_meters_or_nil(nil), do: nil
  def to_kilogram_square_meters_or_nil(unit), do: to_kilogram_square_meters(unit)

  @spec to_newtons_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_newtons_or_nil(nil), do: nil
  def to_newtons_or_nil(unit), do: to_newtons(unit)

  @spec to_newton_meters_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_newton_meters_or_nil(nil), do: nil
  def to_newton_meters_or_nil(unit), do: to_newton_meters(unit)

  @spec to_meters_per_second_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_meters_per_second_or_nil(nil), do: nil
  def to_meters_per_second_or_nil(unit), do: to_meters_per_second(unit)

  @spec to_radians_per_second_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_radians_per_second_or_nil(nil), do: nil
  def to_radians_per_second_or_nil(unit), do: to_radians_per_second(unit)

  @spec to_meters_per_square_second_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_meters_per_square_second_or_nil(nil), do: nil
  def to_meters_per_square_second_or_nil(unit), do: to_meters_per_square_second(unit)

  @spec to_radians_per_square_second_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_radians_per_square_second_or_nil(nil), do: nil
  def to_radians_per_square_second_or_nil(unit), do: to_radians_per_square_second(unit)

  @spec to_linear_damping_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_linear_damping_or_nil(nil), do: nil
  def to_linear_damping_or_nil(unit), do: to_linear_damping(unit)

  @spec to_rotational_damping_or_nil(Localize.Unit.t() | nil) :: float() | nil
  def to_rotational_damping_or_nil(nil), do: nil
  def to_rotational_damping_or_nil(unit), do: to_rotational_damping(unit)
end
