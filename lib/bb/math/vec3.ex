# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Vec3 do
  @moduledoc """
  3D vector backed by an Nx tensor.

  All operations are performed using Nx for consistent performance
  and potential GPU acceleration.

  ## Examples

      iex> v = BB.Math.Vec3.new(1, 2, 3)
      iex> BB.Math.Vec3.x(v)
      1.0

      iex> a = BB.Math.Vec3.new(1, 0, 0)
      iex> b = BB.Math.Vec3.new(0, 1, 0)
      iex> c = BB.Math.Vec3.cross(a, b)
      iex> BB.Math.Vec3.z(c)
      1.0
  """

  defstruct [:tensor]

  @type t :: %__MODULE__{tensor: Nx.Tensor.t()}

  @doc """
  Creates a new vector from x, y, z components.

  ## Examples

      iex> v = BB.Math.Vec3.new(1, 2, 3)
      iex> {BB.Math.Vec3.x(v), BB.Math.Vec3.y(v), BB.Math.Vec3.z(v)}
      {1.0, 2.0, 3.0}
  """
  @spec new(number(), number(), number()) :: t()
  def new(x, y, z) do
    %__MODULE__{tensor: Nx.tensor([x, y, z], type: :f64)}
  end

  @doc """
  Creates a vector from an existing `{3}` tensor.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor) do
    %__MODULE__{tensor: Nx.as_type(tensor, :f64)}
  end

  @doc """
  Returns the zero vector.

  ## Examples

      iex> v = BB.Math.Vec3.zero()
      iex> {BB.Math.Vec3.x(v), BB.Math.Vec3.y(v), BB.Math.Vec3.z(v)}
      {0.0, 0.0, 0.0}
  """
  @spec zero() :: t()
  def zero do
    %__MODULE__{tensor: Nx.tensor([0.0, 0.0, 0.0], type: :f64)}
  end

  @doc "Returns the unit X vector (1, 0, 0)."
  @spec unit_x() :: t()
  def unit_x, do: %__MODULE__{tensor: Nx.tensor([1.0, 0.0, 0.0], type: :f64)}

  @doc "Returns the unit Y vector (0, 1, 0)."
  @spec unit_y() :: t()
  def unit_y, do: %__MODULE__{tensor: Nx.tensor([0.0, 1.0, 0.0], type: :f64)}

  @doc "Returns the unit Z vector (0, 0, 1)."
  @spec unit_z() :: t()
  def unit_z, do: %__MODULE__{tensor: Nx.tensor([0.0, 0.0, 1.0], type: :f64)}

  @doc "Returns the underlying tensor."
  @spec tensor(t()) :: Nx.Tensor.t()
  def tensor(%__MODULE__{tensor: t}), do: t

  @doc "Returns the X component."
  @spec x(t()) :: float()
  def x(%__MODULE__{tensor: t}), do: Nx.to_number(t[0])

  @doc "Returns the Y component."
  @spec y(t()) :: float()
  def y(%__MODULE__{tensor: t}), do: Nx.to_number(t[1])

  @doc "Returns the Z component."
  @spec z(t()) :: float()
  def z(%__MODULE__{tensor: t}), do: Nx.to_number(t[2])

  @doc "Returns the components as a list [x, y, z]."
  @spec to_list(t()) :: [float()]
  def to_list(%__MODULE__{tensor: t}), do: Nx.to_flat_list(t)

  @doc """
  Creates a vector from a list of three numbers.

  ## Examples

      iex> v = BB.Math.Vec3.from_list([1, 2, 3])
      iex> BB.Math.Vec3.to_list(v)
      [1.0, 2.0, 3.0]
  """
  @spec from_list([number()]) :: t()
  def from_list([x, y, z]), do: new(x, y, z)

  @doc """
  Adds two vectors.

  ## Examples

      iex> a = BB.Math.Vec3.new(1, 2, 3)
      iex> b = BB.Math.Vec3.new(4, 5, 6)
      iex> c = BB.Math.Vec3.add(a, b)
      iex> BB.Math.Vec3.to_list(c)
      [5.0, 7.0, 9.0]
  """
  @spec add(t(), t()) :: t()
  def add(%__MODULE__{tensor: a}, %__MODULE__{tensor: b}) do
    %__MODULE__{tensor: Nx.add(a, b)}
  end

  @doc """
  Subtracts vector b from vector a.

  ## Examples

      iex> a = BB.Math.Vec3.new(4, 5, 6)
      iex> b = BB.Math.Vec3.new(1, 2, 3)
      iex> c = BB.Math.Vec3.subtract(a, b)
      iex> BB.Math.Vec3.to_list(c)
      [3.0, 3.0, 3.0]
  """
  @spec subtract(t(), t()) :: t()
  def subtract(%__MODULE__{tensor: a}, %__MODULE__{tensor: b}) do
    %__MODULE__{tensor: Nx.subtract(a, b)}
  end

  @doc """
  Negates a vector.

  ## Examples

      iex> v = BB.Math.Vec3.new(1, -2, 3)
      iex> n = BB.Math.Vec3.negate(v)
      iex> BB.Math.Vec3.to_list(n)
      [-1.0, 2.0, -3.0]
  """
  @spec negate(t()) :: t()
  def negate(%__MODULE__{tensor: t}) do
    %__MODULE__{tensor: Nx.negate(t)}
  end

  @doc """
  Scales a vector by a scalar.

  ## Examples

      iex> v = BB.Math.Vec3.new(1, 2, 3)
      iex> s = BB.Math.Vec3.scale(v, 2)
      iex> BB.Math.Vec3.to_list(s)
      [2.0, 4.0, 6.0]
  """
  @spec scale(t(), number()) :: t()
  def scale(%__MODULE__{tensor: t}, scalar) do
    %__MODULE__{tensor: Nx.multiply(t, scalar)}
  end

  @doc """
  Computes the dot product of two vectors.

  ## Examples

      iex> a = BB.Math.Vec3.new(1, 2, 3)
      iex> b = BB.Math.Vec3.new(4, 5, 6)
      iex> BB.Math.Vec3.dot(a, b)
      32.0
  """
  @spec dot(t(), t()) :: float()
  def dot(%__MODULE__{tensor: a}, %__MODULE__{tensor: b}) do
    Nx.to_number(Nx.dot(a, b))
  end

  @doc """
  Computes the cross product of two vectors.

  ## Examples

      iex> a = BB.Math.Vec3.new(1, 0, 0)
      iex> b = BB.Math.Vec3.new(0, 1, 0)
      iex> c = BB.Math.Vec3.cross(a, b)
      iex> BB.Math.Vec3.to_list(c)
      [0.0, 0.0, 1.0]
  """
  @spec cross(t(), t()) :: t()
  def cross(%__MODULE__{tensor: a}, %__MODULE__{tensor: b}) do
    # Cross product: (a2*b3 - a3*b2, a3*b1 - a1*b3, a1*b2 - a2*b1)
    a1 = a[0]
    a2 = a[1]
    a3 = a[2]
    b1 = b[0]
    b2 = b[1]
    b3 = b[2]

    result =
      Nx.stack([
        Nx.subtract(Nx.multiply(a2, b3), Nx.multiply(a3, b2)),
        Nx.subtract(Nx.multiply(a3, b1), Nx.multiply(a1, b3)),
        Nx.subtract(Nx.multiply(a1, b2), Nx.multiply(a2, b1))
      ])

    %__MODULE__{tensor: result}
  end

  @doc """
  Computes the magnitude (length) of a vector.

  ## Examples

      iex> v = BB.Math.Vec3.new(3, 4, 0)
      iex> BB.Math.Vec3.magnitude(v)
      5.0
  """
  @spec magnitude(t()) :: float()
  def magnitude(%__MODULE__{tensor: t}) do
    Nx.to_number(Nx.sqrt(Nx.dot(t, t)))
  end

  @doc """
  Computes the squared magnitude of a vector.

  More efficient than `magnitude/1` when you only need to compare lengths.

  ## Examples

      iex> v = BB.Math.Vec3.new(3, 4, 0)
      iex> BB.Math.Vec3.magnitude_squared(v)
      25.0
  """
  @spec magnitude_squared(t()) :: float()
  def magnitude_squared(%__MODULE__{tensor: t}) do
    Nx.to_number(Nx.dot(t, t))
  end

  @doc """
  Normalises a vector to unit length.

  Returns zero vector if input has zero magnitude.

  ## Examples

      iex> v = BB.Math.Vec3.new(3, 0, 0)
      iex> n = BB.Math.Vec3.normalise(v)
      iex> BB.Math.Vec3.to_list(n)
      [1.0, 0.0, 0.0]
  """
  @spec normalise(t()) :: t()
  def normalise(%__MODULE__{tensor: t}) do
    mag_sq = Nx.dot(t, t)
    mag = Nx.sqrt(mag_sq)

    # Avoid division by zero - return zero vector if magnitude is zero
    safe_mag = Nx.select(Nx.less(mag, 1.0e-10), Nx.tensor(1.0, type: :f64), mag)
    normalised = Nx.divide(t, safe_mag)

    # If original magnitude was zero, return zero vector
    result = Nx.select(Nx.less(mag, 1.0e-10), Nx.tensor([0.0, 0.0, 0.0], type: :f64), normalised)

    %__MODULE__{tensor: result}
  end

  @doc """
  Computes the distance between two points (as vectors).

  ## Examples

      iex> a = BB.Math.Vec3.new(0, 0, 0)
      iex> b = BB.Math.Vec3.new(3, 4, 0)
      iex> BB.Math.Vec3.distance(a, b)
      5.0
  """
  @spec distance(t(), t()) :: float()
  def distance(%__MODULE__{} = a, %__MODULE__{} = b) do
    subtract(b, a) |> magnitude()
  end

  @doc """
  Linearly interpolates between two vectors.

  ## Examples

      iex> a = BB.Math.Vec3.new(0, 0, 0)
      iex> b = BB.Math.Vec3.new(10, 10, 10)
      iex> c = BB.Math.Vec3.lerp(a, b, 0.5)
      iex> BB.Math.Vec3.to_list(c)
      [5.0, 5.0, 5.0]
  """
  @spec lerp(t(), t(), number()) :: t()
  def lerp(%__MODULE__{tensor: a}, %__MODULE__{tensor: b}, t) do
    # lerp(a, b, t) = a + t * (b - a) = a * (1 - t) + b * t
    result =
      Nx.add(
        Nx.multiply(a, 1 - t),
        Nx.multiply(b, t)
      )

    %__MODULE__{tensor: result}
  end
end
