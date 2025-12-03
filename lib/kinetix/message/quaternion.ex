# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Message.Quaternion do
  @moduledoc """
  Helper functions for quaternion tagged tuples.

  Quaternions are represented as `{:quaternion, x, y, z, w}` where components
  are floats in XYZW order. Used for representing 3D orientations.

  The quaternion should be normalised (magnitude = 1) for rotation purposes,
  but this module does not enforce normalisation.

  ## Examples

      iex> alias Kinetix.Message.Quaternion
      iex> q = Quaternion.identity()
      {:quaternion, 0.0, 0.0, 0.0, 1.0}
      iex> Quaternion.w(q)
      1.0
  """

  @type t :: {:quaternion, float(), float(), float(), float()}

  @doc """
  Create a new quaternion from x, y, z, w components.

  Accepts any numeric type and converts to float.

  ## Examples

      iex> Kinetix.Message.Quaternion.new(0, 0, 0, 1)
      {:quaternion, 0.0, 0.0, 0.0, 1.0}

      iex> Kinetix.Message.Quaternion.new(0.0, 0.707, 0.0, 0.707)
      {:quaternion, 0.0, 0.707, 0.0, 0.707}
  """
  @spec new(number(), number(), number(), number()) :: t()
  def new(x, y, z, w)
      when is_number(x) and is_number(y) and is_number(z) and is_number(w) do
    {:quaternion, x / 1, y / 1, z / 1, w / 1}
  end

  @doc """
  Returns the identity quaternion (no rotation).

  ## Examples

      iex> Kinetix.Message.Quaternion.identity()
      {:quaternion, 0.0, 0.0, 0.0, 1.0}
  """
  @spec identity() :: t()
  def identity, do: {:quaternion, 0.0, 0.0, 0.0, 1.0}

  @doc """
  Get the X component.

  ## Examples

      iex> Kinetix.Message.Quaternion.x({:quaternion, 0.1, 0.2, 0.3, 0.9})
      0.1
  """
  @spec x(t()) :: float()
  def x({:quaternion, x, _y, _z, _w}), do: x

  @doc """
  Get the Y component.

  ## Examples

      iex> Kinetix.Message.Quaternion.y({:quaternion, 0.1, 0.2, 0.3, 0.9})
      0.2
  """
  @spec y(t()) :: float()
  def y({:quaternion, _x, y, _z, _w}), do: y

  @doc """
  Get the Z component.

  ## Examples

      iex> Kinetix.Message.Quaternion.z({:quaternion, 0.1, 0.2, 0.3, 0.9})
      0.3
  """
  @spec z(t()) :: float()
  def z({:quaternion, _x, _y, z, _w}), do: z

  @doc """
  Get the W (scalar) component.

  ## Examples

      iex> Kinetix.Message.Quaternion.w({:quaternion, 0.1, 0.2, 0.3, 0.9})
      0.9
  """
  @spec w(t()) :: float()
  def w({:quaternion, _x, _y, _z, w}), do: w

  @doc """
  Convert to a list [x, y, z, w].

  ## Examples

      iex> Kinetix.Message.Quaternion.to_list({:quaternion, 0.0, 0.0, 0.0, 1.0})
      [0.0, 0.0, 0.0, 1.0]
  """
  @spec to_list(t()) :: [float()]
  def to_list({:quaternion, x, y, z, w}), do: [x, y, z, w]

  @doc """
  Create from a list [x, y, z, w].

  ## Examples

      iex> Kinetix.Message.Quaternion.from_list([0.0, 0.0, 0.0, 1.0])
      {:quaternion, 0.0, 0.0, 0.0, 1.0}
  """
  @spec from_list([number()]) :: t()
  def from_list([x, y, z, w]), do: new(x, y, z, w)
end
