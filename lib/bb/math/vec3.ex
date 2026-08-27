# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Vec3 do
  @moduledoc """
  A 3D vector, held as three `:f64` floats.

  ## Not tensor-backed

  Like `BB.Math.Transform2D`, and unlike a batched computation, a single 3-vector
  is far cheaper as three BEAM floats than as an `Nx` tensor: eager per-op
  dispatch on three elements costs orders of magnitude more than the arithmetic
  it performs, and allocates a tensor per intermediate. Vectors are built and
  read on every sensor sample, so that cost lands squarely in the hot path.

  `tensor/1` and `from_tensor/1` convert at the boundary of code that genuinely
  wants tensors - batched forward kinematics, the IK solvers, `Nx.LinAlg`. Those
  callers build one tensor for a whole computation rather than one per operation,
  which is where `Nx` earns its keep.

  ## Examples

      iex> BB.Math.Vec3.new(1, 2, 3)
      BB.Math.Vec3.new(1.0, 2.0, 3.0)

      iex> v = BB.Math.Vec3.new(1, 2, 3)
      iex> BB.Math.Vec3.x(v)
      1.0

      iex> a = BB.Math.Vec3.new(1, 0, 0)
      iex> b = BB.Math.Vec3.new(0, 1, 0)
      iex> c = BB.Math.Vec3.cross(a, b)
      iex> BB.Math.Vec3.z(c)
      1.0
  """

  defstruct [:x, :y, :z]

  @type t :: %__MODULE__{x: float(), y: float(), z: float()}

  @zero_threshold 1.0e-10

  defimpl Inspect do
    import Inspect.Algebra

    @spec inspect(@for.t(), Inspect.Opts.t()) :: Inspect.Algebra.t()
    def inspect(%@for{x: x, y: y, z: z}, opts) do
      container_doc("BB.Math.Vec3.new(", [x, y, z], ")", opts, &to_doc/2)
    end
  end

  @doc """
  Creates a new vector from x, y, z components.

  ## Examples

      iex> v = BB.Math.Vec3.new(1, 2, 3)
      iex> {BB.Math.Vec3.x(v), BB.Math.Vec3.y(v), BB.Math.Vec3.z(v)}
      {1.0, 2.0, 3.0}
  """
  @spec new(number(), number(), number()) :: t()
  def new(x, y, z) do
    %__MODULE__{x: x / 1, y: y / 1, z: z / 1}
  end

  @doc """
  Creates a vector from an existing `{3}` tensor.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor) do
    [x, y, z] = tensor |> Nx.as_type(:f64) |> Nx.to_flat_list()

    %__MODULE__{x: x, y: y, z: z}
  end

  @doc """
  Returns the zero vector.

  ## Examples

      iex> v = BB.Math.Vec3.zero()
      iex> {BB.Math.Vec3.x(v), BB.Math.Vec3.y(v), BB.Math.Vec3.z(v)}
      {0.0, 0.0, 0.0}
  """
  @spec zero() :: t()
  def zero, do: %__MODULE__{x: 0.0, y: 0.0, z: 0.0}

  @doc "Returns the unit X vector (1, 0, 0)."
  @spec unit_x() :: t()
  def unit_x, do: %__MODULE__{x: 1.0, y: 0.0, z: 0.0}

  @doc "Returns the unit Y vector (0, 1, 0)."
  @spec unit_y() :: t()
  def unit_y, do: %__MODULE__{x: 0.0, y: 1.0, z: 0.0}

  @doc "Returns the unit Z vector (0, 0, 1)."
  @spec unit_z() :: t()
  def unit_z, do: %__MODULE__{x: 0.0, y: 0.0, z: 1.0}

  @doc "Returns the vector as a `{3}` `:f64` tensor."
  @spec tensor(t()) :: Nx.Tensor.t()
  def tensor(%__MODULE__{x: x, y: y, z: z}), do: Nx.tensor([x, y, z], type: :f64)

  @doc "Returns the X component."
  @spec x(t()) :: float()
  def x(%__MODULE__{x: x}), do: x

  @doc "Returns the Y component."
  @spec y(t()) :: float()
  def y(%__MODULE__{y: y}), do: y

  @doc "Returns the Z component."
  @spec z(t()) :: float()
  def z(%__MODULE__{z: z}), do: z

  @doc "Returns the components as a list [x, y, z]."
  @spec to_list(t()) :: [float()]
  def to_list(%__MODULE__{x: x, y: y, z: z}), do: [x, y, z]

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
  def add(%__MODULE__{} = a, %__MODULE__{} = b) do
    %__MODULE__{x: a.x + b.x, y: a.y + b.y, z: a.z + b.z}
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
  def subtract(%__MODULE__{} = a, %__MODULE__{} = b) do
    %__MODULE__{x: a.x - b.x, y: a.y - b.y, z: a.z - b.z}
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
  def negate(%__MODULE__{x: x, y: y, z: z}) do
    %__MODULE__{x: -x, y: -y, z: -z}
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
  def scale(%__MODULE__{x: x, y: y, z: z}, scalar) do
    %__MODULE__{x: x * scalar, y: y * scalar, z: z * scalar}
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
  def dot(%__MODULE__{} = a, %__MODULE__{} = b) do
    a.x * b.x + a.y * b.y + a.z * b.z
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
  def cross(%__MODULE__{} = a, %__MODULE__{} = b) do
    %__MODULE__{
      x: a.y * b.z - a.z * b.y,
      y: a.z * b.x - a.x * b.z,
      z: a.x * b.y - a.y * b.x
    }
  end

  @doc """
  Computes the magnitude (length) of a vector.

  ## Examples

      iex> v = BB.Math.Vec3.new(3, 4, 0)
      iex> BB.Math.Vec3.magnitude(v)
      5.0
  """
  @spec magnitude(t()) :: float()
  def magnitude(%__MODULE__{} = v) do
    :math.sqrt(magnitude_squared(v))
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
  def magnitude_squared(%__MODULE__{x: x, y: y, z: z}) do
    x * x + y * y + z * z
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
  def normalise(%__MODULE__{x: x, y: y, z: z} = v) do
    case magnitude(v) do
      magnitude when magnitude < @zero_threshold -> zero()
      magnitude -> %__MODULE__{x: x / magnitude, y: y / magnitude, z: z / magnitude}
    end
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
  def lerp(%__MODULE__{} = a, %__MODULE__{} = b, t) do
    %__MODULE__{
      x: a.x * (1 - t) + b.x * t,
      y: a.y * (1 - t) + b.y * t,
      z: a.z * (1 - t) + b.z * t
    }
  end
end
