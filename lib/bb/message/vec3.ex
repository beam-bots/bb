# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Vec3 do
  @moduledoc """
  Helper functions for 3D vector tagged tuples.

  Vectors are represented as `{:vec3, x, y, z}` where x, y, z are floats.
  Used for positions (metres), velocities (m/s), forces (N), and other
  3D quantities. The meaning depends on context.

  All values are in base SI units - no unit conversion is performed.

  ## Examples

      iex> alias BB.Message.Vec3
      iex> pos = Vec3.new(1.0, 2.0, 3.0)
      {:vec3, 1.0, 2.0, 3.0}
      iex> Vec3.x(pos)
      1.0
  """

  @type t :: {:vec3, float(), float(), float()}

  @doc """
  Create a new vec3 from x, y, z components.

  Accepts any numeric type and converts to float.

  ## Examples

      iex> BB.Message.Vec3.new(1, 2, 3)
      {:vec3, 1.0, 2.0, 3.0}

      iex> BB.Message.Vec3.new(1.5, 2.5, 3.5)
      {:vec3, 1.5, 2.5, 3.5}
  """
  @spec new(number(), number(), number()) :: t()
  def new(x, y, z) when is_number(x) and is_number(y) and is_number(z) do
    {:vec3, x / 1, y / 1, z / 1}
  end

  @doc """
  Returns the zero vector.

  ## Examples

      iex> BB.Message.Vec3.zero()
      {:vec3, 0.0, 0.0, 0.0}
  """
  @spec zero() :: t()
  def zero, do: {:vec3, 0.0, 0.0, 0.0}

  @doc """
  Returns a unit vector along the X axis.

  ## Examples

      iex> BB.Message.Vec3.unit_x()
      {:vec3, 1.0, 0.0, 0.0}
  """
  @spec unit_x() :: t()
  def unit_x, do: {:vec3, 1.0, 0.0, 0.0}

  @doc """
  Returns a unit vector along the Y axis.

  ## Examples

      iex> BB.Message.Vec3.unit_y()
      {:vec3, 0.0, 1.0, 0.0}
  """
  @spec unit_y() :: t()
  def unit_y, do: {:vec3, 0.0, 1.0, 0.0}

  @doc """
  Returns a unit vector along the Z axis.

  ## Examples

      iex> BB.Message.Vec3.unit_z()
      {:vec3, 0.0, 0.0, 1.0}
  """
  @spec unit_z() :: t()
  def unit_z, do: {:vec3, 0.0, 0.0, 1.0}

  @doc """
  Get the X component.

  ## Examples

      iex> BB.Message.Vec3.x({:vec3, 1.0, 2.0, 3.0})
      1.0
  """
  @spec x(t()) :: float()
  def x({:vec3, x, _y, _z}), do: x

  @doc """
  Get the Y component.

  ## Examples

      iex> BB.Message.Vec3.y({:vec3, 1.0, 2.0, 3.0})
      2.0
  """
  @spec y(t()) :: float()
  def y({:vec3, _x, y, _z}), do: y

  @doc """
  Get the Z component.

  ## Examples

      iex> BB.Message.Vec3.z({:vec3, 1.0, 2.0, 3.0})
      3.0
  """
  @spec z(t()) :: float()
  def z({:vec3, _x, _y, z}), do: z

  @doc """
  Convert to a list [x, y, z].

  ## Examples

      iex> BB.Message.Vec3.to_list({:vec3, 1.0, 2.0, 3.0})
      [1.0, 2.0, 3.0]
  """
  @spec to_list(t()) :: [float()]
  def to_list({:vec3, x, y, z}), do: [x, y, z]

  @doc """
  Create from a list [x, y, z].

  ## Examples

      iex> BB.Message.Vec3.from_list([1.0, 2.0, 3.0])
      {:vec3, 1.0, 2.0, 3.0}
  """
  @spec from_list([number()]) :: t()
  def from_list([x, y, z]), do: new(x, y, z)
end
