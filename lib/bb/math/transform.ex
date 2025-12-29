# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Transform do
  @moduledoc """
  Homogeneous transformation matrices for 3D transformations, backed by an Nx tensor.

  All transforms are represented as 4x4 matrices in row-major order:

  ```
  | R11 R12 R13 Tx |
  | R21 R22 R23 Ty |
  | R31 R32 R33 Tz |
  |  0   0   0   1 |
  ```

  Where the upper-left 3x3 is the rotation matrix and the rightmost column
  is the translation vector.

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

  defstruct [:tensor]

  @type t :: %__MODULE__{tensor: Nx.Tensor.t()}

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
  def identity do
    %__MODULE__{tensor: Nx.eye(4, type: :f64)}
  end

  @doc """
  Creates a transform from an existing `{4, 4}` tensor.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor) do
    %__MODULE__{tensor: Nx.as_type(tensor, :f64)}
  end

  @doc """
  Returns the underlying `{4, 4}` tensor.
  """
  @spec tensor(t()) :: Nx.Tensor.t()
  def tensor(%__MODULE__{tensor: t}), do: t

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
  def translation(%Vec3{} = v) do
    x = Vec3.x(v)
    y = Vec3.y(v)
    z = Vec3.z(v)

    %__MODULE__{
      tensor:
        Nx.tensor(
          [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0]
          ],
          type: :f64
        )
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
      tensor:
        Nx.tensor(
          [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, -s, 0.0],
            [0.0, s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0]
          ],
          type: :f64
        )
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
      tensor:
        Nx.tensor(
          [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0]
          ],
          type: :f64
        )
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
      tensor:
        Nx.tensor(
          [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
          ],
          type: :f64
        )
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
  def compose(%__MODULE__{tensor: a}, %__MODULE__{tensor: b}) do
    %__MODULE__{tensor: Nx.dot(a, b)}
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
  def get_translation(%__MODULE__{tensor: tensor}) do
    Vec3.from_tensor(Nx.slice(tensor, [0, 3], [3, 1]) |> Nx.reshape({3}))
  end

  @doc """
  Get the rotation matrix (3x3) from a transform.
  """
  @spec get_rotation(t()) :: Nx.Tensor.t()
  def get_rotation(%__MODULE__{tensor: tensor}) do
    tensor[0..2][0..2]
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
  def apply_to_point(%__MODULE__{tensor: tensor}, %Vec3{tensor: v}) do
    point = Nx.concatenate([v, Nx.tensor([1.0], type: :f64)])
    result = Nx.dot(tensor, point)
    Vec3.from_tensor(Nx.slice(result, [0], [3]))
  end

  @doc """
  Compute the inverse of a transformation matrix.

  For a valid transformation matrix, this computes the inverse transform.
  """
  @spec inverse(t()) :: t()
  def inverse(%__MODULE__{tensor: tensor}) do
    r = get_rotation(%__MODULE__{tensor: tensor})
    t_vec = get_translation(%__MODULE__{tensor: tensor})

    r_inv = Nx.transpose(r)
    t_inv = Vec3.negate(Vec3.from_tensor(Nx.dot(r_inv, Vec3.tensor(t_vec))))

    %__MODULE__{
      tensor:
        Nx.tensor(
          [
            [
              Nx.to_number(r_inv[0][0]),
              Nx.to_number(r_inv[0][1]),
              Nx.to_number(r_inv[0][2]),
              Vec3.x(t_inv)
            ],
            [
              Nx.to_number(r_inv[1][0]),
              Nx.to_number(r_inv[1][1]),
              Nx.to_number(r_inv[1][2]),
              Vec3.y(t_inv)
            ],
            [
              Nx.to_number(r_inv[2][0]),
              Nx.to_number(r_inv[2][1]),
              Nx.to_number(r_inv[2][2]),
              Vec3.z(t_inv)
            ],
            [0.0, 0.0, 0.0, 1.0]
          ],
          type: :f64
        )
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
  def from_axis_angle(%Vec3{} = axis, angle) do
    ax = Vec3.x(axis)
    ay = Vec3.y(axis)
    az = Vec3.z(axis)

    c = :math.cos(angle)
    s = :math.sin(angle)
    t = 1.0 - c

    %__MODULE__{
      tensor:
        Nx.tensor(
          [
            [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
            [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
            [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
            [0.0, 0.0, 0.0, 1.0]
          ],
          type: :f64
        )
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
    rot_3x3 = Quaternion.to_rotation_matrix(q)

    row0 = Nx.concatenate([rot_3x3[0], Nx.tensor([0.0], type: :f64)])
    row1 = Nx.concatenate([rot_3x3[1], Nx.tensor([0.0], type: :f64)])
    row2 = Nx.concatenate([rot_3x3[2], Nx.tensor([0.0], type: :f64)])
    row3 = Nx.tensor([0.0, 0.0, 0.0, 1.0], type: :f64)

    %__MODULE__{tensor: Nx.stack([row0, row1, row2, row3])}
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
    rot_3x3 = get_rotation(transform)
    Quaternion.from_rotation_matrix(rot_3x3)
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
  def from_position_quaternion(%Vec3{} = pos, %Quaternion{} = q) do
    rot_3x3 = Quaternion.to_rotation_matrix(q)
    pos_tensor = Vec3.tensor(pos)

    row0 = Nx.concatenate([rot_3x3[0], Nx.reshape(pos_tensor[0], {1})])
    row1 = Nx.concatenate([rot_3x3[1], Nx.reshape(pos_tensor[1], {1})])
    row2 = Nx.concatenate([rot_3x3[2], Nx.reshape(pos_tensor[2], {1})])
    row3 = Nx.tensor([0.0, 0.0, 0.0, 1.0], type: :f64)

    %__MODULE__{tensor: Nx.stack([row0, row1, row2, row3])}
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
  def get_forward_vector(%__MODULE__{tensor: tensor}) do
    Vec3.new(
      Nx.to_number(tensor[0][2]),
      Nx.to_number(tensor[1][2]),
      Nx.to_number(tensor[2][2])
    )
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
  def get_up_vector(%__MODULE__{tensor: tensor}) do
    Vec3.new(
      Nx.to_number(tensor[0][1]),
      Nx.to_number(tensor[1][1]),
      Nx.to_number(tensor[2][1])
    )
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
  def get_right_vector(%__MODULE__{tensor: tensor}) do
    Vec3.new(
      Nx.to_number(tensor[0][0]),
      Nx.to_number(tensor[1][0]),
      Nx.to_number(tensor[2][0])
    )
  end
end
