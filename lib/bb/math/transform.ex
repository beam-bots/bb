# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Transform do
  @moduledoc """
  Homogeneous transformation matrices for 3D transformations.

  All transforms are represented as 4x4 matrices in row-major order:

  ```
  | R11 R12 R13 Tx |
  | R21 R22 R23 Ty |
  | R31 R32 R33 Tz |
  |  0   0   0   1 |
  ```

  Where the upper-left 3x3 is the rotation matrix and the rightmost column
  is the translation vector.

  ## Not tensor-backed

  The sixteen entries are held as a flat row-major tuple of floats rather than
  an `Nx` tensor, for the reasons in `BB.Math.Vec3` - a 4x4 compose is 64
  multiply-adds, and eager `Nx` dispatch costs far more than that. A tuple also
  destructures in a single pattern match, so `compose/2` reads its 32 inputs
  without a map lookup each.

  `tensor/1` and `from_tensor/1` convert at the boundary of `BB.Robot.Kinematics`
  and the IK solvers, where a whole chain is walked in one batched `defn` and
  `Nx` genuinely pays for itself.

  ## Conventions

  - All angles are in radians
  - All distances are in metres
  - Rotations use XYZ Euler angles (roll-pitch-yaw)
  - Coordinate frame follows right-hand rule

  ## Examples

      iex> t = BB.Math.Transform.identity()
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [0.0, 0.0, 0.0]

      iex> t = BB.Math.Transform.translation(BB.Math.Vec3.new(1, 2, 3))
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 2.0, 3.0]
  """

  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  defstruct [:m]

  @typedoc "The sixteen matrix entries, row-major."
  @type elements ::
          {float(), float(), float(), float(), float(), float(), float(), float(), float(),
           float(), float(), float(), float(), float(), float(), float()}

  @type t :: %__MODULE__{m: elements()}

  @identity {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}

  @doc """
  Create a 4x4 identity transformation matrix.

  ## Examples

      iex> t = BB.Math.Transform.identity()
      iex> BB.Math.Transform.tensor(t) |> Nx.to_list()
      [[1.0, 0.0, 0.0, 0.0],
       [0.0, 1.0, 0.0, 0.0],
       [0.0, 0.0, 1.0, 0.0],
       [0.0, 0.0, 0.0, 1.0]]
  """
  @spec identity() :: t()
  def identity, do: %__MODULE__{m: @identity}

  @doc """
  Creates a transform from an existing `{4, 4}` tensor.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor) do
    %__MODULE__{
      m: tensor |> Nx.as_type(:f64) |> Nx.to_flat_list() |> List.to_tuple()
    }
  end

  @doc """
  Returns the transform as a `{4, 4}` `:f64` tensor.
  """
  @spec tensor(t()) :: Nx.Tensor.t()
  def tensor(%__MODULE__{m: m}) do
    m |> Tuple.to_list() |> Enum.chunk_every(4) |> Nx.tensor(type: :f64)
  end

  @doc """
  Returns the sixteen matrix entries as a flat row-major list.
  """
  @spec to_list(t()) :: [float()]
  def to_list(%__MODULE__{m: m}), do: Tuple.to_list(m)

  @doc """
  Create a transformation matrix from position and orientation.

  The origin map should have:
  - `position`: {x, y, z} in metres
  - `orientation`: {roll, pitch, yaw} in radians

  Rotation is applied in XYZ order (roll around X, then pitch around Y,
  then yaw around Z).

  ## Examples

      iex> origin = %{position: {1.0, 2.0, 3.0}, orientation: {0.0, 0.0, 0.0}}
      iex> t = BB.Math.Transform.from_origin(origin)
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 2.0, 3.0]
  """
  @spec from_origin(%{
          position: {float(), float(), float()},
          orientation: {float(), float(), float()}
        }) :: t()
  def from_origin(%{position: {x, y, z}, orientation: {roll, pitch, yaw}}) do
    rotation_x(roll)
    |> compose(rotation_y(pitch))
    |> compose(rotation_z(yaw))
    |> compose(translation(Vec3.new(x, y, z)))
  end

  def from_origin(nil), do: identity()

  @doc """
  Create a pure translation matrix from a Vec3.

  ## Examples

      iex> t = BB.Math.Transform.translation(BB.Math.Vec3.new(1, 2, 3))
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 2.0, 3.0]
  """
  @spec translation(Vec3.t()) :: t()
  def translation(%Vec3{x: x, y: y, z: z}) do
    %__MODULE__{
      m: {
        1.0,
        0.0,
        0.0,
        x,
        0.0,
        1.0,
        0.0,
        y,
        0.0,
        0.0,
        1.0,
        z,
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Create a rotation matrix around the X axis (roll).

  ## Examples

      iex> t = BB.Math.Transform.rotation_x(:math.pi() / 2)
      iex> v = BB.Math.Transform.apply_to_point(t, BB.Math.Vec3.new(0, 1, 0))
      iex> Float.round(BB.Math.Vec3.z(v), 6)
      1.0
  """
  @spec rotation_x(float()) :: t()
  def rotation_x(angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    %__MODULE__{
      m: {
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        c,
        -s,
        0.0,
        0.0,
        s,
        c,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Create a rotation matrix around the Y axis (pitch).
  """
  @spec rotation_y(float()) :: t()
  def rotation_y(angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    %__MODULE__{
      m: {
        c,
        0.0,
        s,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        -s,
        0.0,
        c,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Create a rotation matrix around the Z axis (yaw).
  """
  @spec rotation_z(float()) :: t()
  def rotation_z(angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    %__MODULE__{
      m: {
        c,
        -s,
        0.0,
        0.0,
        s,
        c,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Compose (multiply) two transformation matrices.

  `compose(a, b)` returns the transform that applies `a` first, then `b`.

  ## Examples

      iex> t1 = BB.Math.Transform.translation(BB.Math.Vec3.new(1, 0, 0))
      iex> t2 = BB.Math.Transform.translation(BB.Math.Vec3.new(0, 2, 0))
      iex> t = BB.Math.Transform.compose(t1, t2)
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 2.0, 0.0]
  """
  @spec compose(t(), t()) :: t()
  def compose(
        %__MODULE__{
          m: {a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33}
        },
        %__MODULE__{
          m: {b00, b01, b02, b03, b10, b11, b12, b13, b20, b21, b22, b23, b30, b31, b32, b33}
        }
      ) do
    %__MODULE__{
      m: {
        a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30,
        a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31,
        a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32,
        a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33,
        a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30,
        a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31,
        a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32,
        a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33,
        a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30,
        a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31,
        a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32,
        a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33,
        a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30,
        a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31,
        a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32,
        a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33
      }
    }
  end

  @doc """
  Compose a list of transforms in order.

  ## Examples

      iex> transforms = [
      ...>   BB.Math.Transform.translation(BB.Math.Vec3.new(1, 0, 0)),
      ...>   BB.Math.Transform.translation(BB.Math.Vec3.new(0, 1, 0)),
      ...>   BB.Math.Transform.translation(BB.Math.Vec3.new(0, 0, 1))
      ...> ]
      iex> t = BB.Math.Transform.compose_all(transforms)
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 1.0, 1.0]
  """
  @spec compose_all([t()]) :: t()
  def compose_all([]), do: identity()
  def compose_all([t]), do: t
  def compose_all([h | t]), do: Enum.reduce(t, h, &compose(&2, &1))

  @doc """
  Get the translation component of a transform as a Vec3.
  """
  @spec get_translation(t()) :: Vec3.t()
  def get_translation(%__MODULE__{m: {_, _, _, x, _, _, _, y, _, _, _, z, _, _, _, _}}) do
    %Vec3{x: x, y: y, z: z}
  end

  @doc """
  Get the rotation matrix (3x3) from a transform.
  """
  @spec get_rotation(t()) :: Nx.Tensor.t()
  def get_rotation(%__MODULE__{} = transform) do
    Nx.tensor(get_rotation_list(transform), type: :f64)
  end

  @doc """
  Get the rotation matrix (3x3) from a transform as a list of rows.
  """
  @spec get_rotation_list(t()) :: [[float()]]
  def get_rotation_list(%__MODULE__{
        m: {r00, r01, r02, _, r10, r11, r12, _, r20, r21, r22, _, _, _, _, _}
      }) do
    [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
  end

  @doc """
  Apply a transform to a 3D point, returning the transformed point.

  ## Examples

      iex> t = BB.Math.Transform.translation(BB.Math.Vec3.new(1, 2, 3))
      iex> p = BB.Math.Transform.apply_to_point(t, BB.Math.Vec3.zero())
      iex> BB.Math.Vec3.to_list(p)
      [1.0, 2.0, 3.0]
  """
  @spec apply_to_point(t(), Vec3.t()) :: Vec3.t()
  def apply_to_point(
        %__MODULE__{m: {m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, _, _, _, _}},
        %Vec3{x: x, y: y, z: z}
      ) do
    %Vec3{
      x: m00 * x + m01 * y + m02 * z + m03,
      y: m10 * x + m11 * y + m12 * z + m13,
      z: m20 * x + m21 * y + m22 * z + m23
    }
  end

  @doc """
  Compute the inverse of a transformation matrix.

  For a valid transformation matrix, this computes the inverse transform.
  """
  @spec inverse(t()) :: t()
  def inverse(%__MODULE__{
        m: {r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz, _, _, _, _}
      }) do
    # Rigid-body inverse: the rotation transposes, and the translation is the
    # transposed rotation applied to the negated original.
    %__MODULE__{
      m: {
        r00,
        r10,
        r20,
        -(r00 * tx + r10 * ty + r20 * tz),
        r01,
        r11,
        r21,
        -(r01 * tx + r11 * ty + r21 * tz),
        r02,
        r12,
        r22,
        -(r02 * tx + r12 * ty + r22 * tz),
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Create a rotation transform around an arbitrary axis using the axis-angle representation.

  Uses Rodrigues' rotation formula to compute the rotation matrix.

  ## Parameters

  - `axis`: normalised axis Vec3
  - `angle`: rotation angle in radians

  ## Examples

      iex> axis = BB.Math.Vec3.unit_z()
      iex> t = BB.Math.Transform.from_axis_angle(axis, :math.pi() / 2)
      iex> p = BB.Math.Transform.apply_to_point(t, BB.Math.Vec3.unit_x())
      iex> {Float.round(BB.Math.Vec3.x(p), 6), Float.round(BB.Math.Vec3.y(p), 6)}
      {0.0, 1.0}
  """
  @spec from_axis_angle(Vec3.t(), float()) :: t()
  def from_axis_angle(%Vec3{x: ax, y: ay, z: az}, angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)
    t = 1.0 - c

    %__MODULE__{
      m: {
        t * ax * ax + c,
        t * ax * ay - s * az,
        t * ax * az + s * ay,
        0.0,
        t * ax * ay + s * az,
        t * ay * ay + c,
        t * ay * az - s * ax,
        0.0,
        t * ax * az - s * ay,
        t * ay * az + s * ax,
        t * az * az + c,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Create a translation transform along an arbitrary axis.

  ## Parameters

  - `axis`: normalised axis Vec3
  - `distance`: translation distance in metres

  ## Examples

      iex> axis = BB.Math.Vec3.unit_x()
      iex> t = BB.Math.Transform.translation_along(axis, 2.5)
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [2.5, 0.0, 0.0]
  """
  @spec translation_along(Vec3.t(), float()) :: t()
  def translation_along(%Vec3{} = axis, distance) do
    translation(Vec3.scale(axis, distance))
  end

  @doc """
  Create a 4x4 transformation matrix from a quaternion (rotation only).

  The resulting matrix has the quaternion's rotation in the upper-left 3x3
  and zero translation.

  ## Examples

      iex> q = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> t = BB.Math.Transform.from_quaternion(q)
      iex> p = BB.Math.Transform.apply_to_point(t, BB.Math.Vec3.unit_x())
      iex> {Float.round(BB.Math.Vec3.x(p), 6), Float.round(BB.Math.Vec3.y(p), 6)}
      {0.0, 1.0}
  """
  @spec from_quaternion(Quaternion.t()) :: t()
  def from_quaternion(%Quaternion{} = q) do
    from_position_quaternion(Vec3.zero(), q)
  end

  @doc """
  Extract a quaternion from a transform.

  Extracts the 3x3 rotation portion and converts it to a unit quaternion.

  ## Examples

      iex> t = BB.Math.Transform.rotation_z(:math.pi() / 2)
      iex> q = BB.Math.Transform.get_quaternion(t)
      iex> {_axis, angle} = BB.Math.Quaternion.to_axis_angle(q)
      iex> Float.round(angle, 6)
      1.570796
  """
  @spec get_quaternion(t()) :: Quaternion.t()
  def get_quaternion(%__MODULE__{} = transform) do
    transform |> get_rotation_list() |> Quaternion.from_rotation_matrix()
  end

  @doc """
  Create a 4x4 transformation matrix from position and quaternion orientation.

  ## Examples

      iex> pos = BB.Math.Vec3.new(1, 2, 3)
      iex> q = BB.Math.Quaternion.identity()
      iex> t = BB.Math.Transform.from_position_quaternion(pos, q)
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 2.0, 3.0]
  """
  @spec from_position_quaternion(Vec3.t(), Quaternion.t()) :: t()
  def from_position_quaternion(%Vec3{x: x, y: y, z: z}, %Quaternion{} = q) do
    [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = Quaternion.to_rotation_list(q)

    %__MODULE__{
      m: {
        r00,
        r01,
        r02,
        x,
        r10,
        r11,
        r12,
        y,
        r20,
        r21,
        r22,
        z,
        0.0,
        0.0,
        0.0,
        1.0
      }
    }
  end

  @doc """
  Get the forward vector (Z-axis) from a transformation matrix.

  The forward vector is the third column of the rotation matrix,
  representing the direction the local Z-axis points in world coordinates.

  ## Examples

      iex> t = BB.Math.Transform.identity()
      iex> fwd = BB.Math.Transform.get_forward_vector(t)
      iex> BB.Math.Vec3.to_list(fwd)
      [0.0, 0.0, 1.0]
  """
  @spec get_forward_vector(t()) :: Vec3.t()
  def get_forward_vector(%__MODULE__{m: {_, _, x, _, _, _, y, _, _, _, z, _, _, _, _, _}}) do
    %Vec3{x: x, y: y, z: z}
  end

  @doc """
  Get the up vector (Y-axis) from a transformation matrix.

  The up vector is the second column of the rotation matrix,
  representing the direction the local Y-axis points in world coordinates.

  ## Examples

      iex> t = BB.Math.Transform.identity()
      iex> up = BB.Math.Transform.get_up_vector(t)
      iex> BB.Math.Vec3.to_list(up)
      [0.0, 1.0, 0.0]
  """
  @spec get_up_vector(t()) :: Vec3.t()
  def get_up_vector(%__MODULE__{m: {_, x, _, _, _, y, _, _, _, z, _, _, _, _, _, _}}) do
    %Vec3{x: x, y: y, z: z}
  end

  @doc """
  Get the right vector (X-axis) from a transformation matrix.

  The right vector is the first column of the rotation matrix,
  representing the direction the local X-axis points in world coordinates.

  ## Examples

      iex> t = BB.Math.Transform.identity()
      iex> right = BB.Math.Transform.get_right_vector(t)
      iex> BB.Math.Vec3.to_list(right)
      [1.0, 0.0, 0.0]
  """
  @spec get_right_vector(t()) :: Vec3.t()
  def get_right_vector(%__MODULE__{m: {x, _, _, _, y, _, _, _, z, _, _, _, _, _, _, _}}) do
    %Vec3{x: x, y: y, z: z}
  end
end
