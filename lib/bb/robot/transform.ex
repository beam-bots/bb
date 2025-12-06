# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Transform do
  @moduledoc """
  Homogeneous transformation matrices for robot kinematics.

  All transforms are represented as 4x4 Nx tensors in row-major order:

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
  - All distances are in meters
  - Rotations use XYZ Euler angles (roll-pitch-yaw)
  - Coordinate frame follows right-hand rule
  """

  @doc """
  Create a 4x4 identity transformation matrix.

  ## Examples

      iex> BB.Robot.Transform.identity() |> Nx.to_list()
      [[1.0, 0.0, 0.0, 0.0],
       [0.0, 1.0, 0.0, 0.0],
       [0.0, 0.0, 1.0, 0.0],
       [0.0, 0.0, 0.0, 1.0]]
  """
  @spec identity() :: Nx.Tensor.t()
  def identity do
    Nx.eye(4, type: :f64)
  end

  @doc """
  Create a transformation matrix from position and orientation.

  The origin map should have:
  - `position`: {x, y, z} in meters
  - `orientation`: {roll, pitch, yaw} in radians

  Rotation is applied in XYZ order (roll around X, then pitch around Y,
  then yaw around Z).

  ## Examples

      iex> origin = %{position: {1.0, 2.0, 3.0}, orientation: {0.0, 0.0, 0.0}}
      iex> t = BB.Robot.Transform.from_origin(origin)
      iex> BB.Robot.Transform.get_translation(t)
      {1.0, 2.0, 3.0}
  """
  @spec from_origin(%{
          position: {float(), float(), float()},
          orientation: {float(), float(), float()}
        }) ::
          Nx.Tensor.t()
  def from_origin(%{position: {x, y, z}, orientation: {roll, pitch, yaw}}) do
    rotation_x(roll)
    |> compose(rotation_y(pitch))
    |> compose(rotation_z(yaw))
    |> compose(translation(x, y, z))
  end

  def from_origin(nil), do: identity()

  @doc """
  Create a pure translation matrix.

  ## Examples

      iex> t = BB.Robot.Transform.translation(1.0, 2.0, 3.0)
      iex> BB.Robot.Transform.get_translation(t)
      {1.0, 2.0, 3.0}
  """
  @spec translation(float(), float(), float()) :: Nx.Tensor.t()
  def translation(x, y, z) do
    Nx.tensor(
      [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  @doc """
  Create a rotation matrix around the X axis (roll).

  ## Examples

      iex> t = BB.Robot.Transform.rotation_x(:math.pi() / 2)
      iex> {_, _, z} = BB.Robot.Transform.apply_to_point(t, {0.0, 1.0, 0.0})
      iex> Float.round(z, 6)
      1.0
  """
  @spec rotation_x(float()) :: Nx.Tensor.t()
  def rotation_x(angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    Nx.tensor(
      [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  @doc """
  Create a rotation matrix around the Y axis (pitch).
  """
  @spec rotation_y(float()) :: Nx.Tensor.t()
  def rotation_y(angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    # Standard right-hand rule: +θ around Y takes X toward -Z
    # (viewing from +Y toward origin, counterclockwise rotation)
    Nx.tensor(
      [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  @doc """
  Create a rotation matrix around the Z axis (yaw).
  """
  @spec rotation_z(float()) :: Nx.Tensor.t()
  def rotation_z(angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    Nx.tensor(
      [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  @doc """
  Compose (multiply) two transformation matrices.

  `compose(a, b)` returns the transform that applies `a` first, then `b`.

  ## Examples

      iex> t1 = BB.Robot.Transform.translation(1.0, 0.0, 0.0)
      iex> t2 = BB.Robot.Transform.translation(0.0, 2.0, 0.0)
      iex> t = BB.Robot.Transform.compose(t1, t2)
      iex> BB.Robot.Transform.get_translation(t)
      {1.0, 2.0, 0.0}
  """
  @spec compose(Nx.Tensor.t(), Nx.Tensor.t()) :: Nx.Tensor.t()
  def compose(transform_a, transform_b) do
    Nx.dot(transform_a, transform_b)
  end

  @doc """
  Compose a list of transforms in order.

  ## Examples

      iex> transforms = [
      ...>   BB.Robot.Transform.translation(1.0, 0.0, 0.0),
      ...>   BB.Robot.Transform.translation(0.0, 1.0, 0.0),
      ...>   BB.Robot.Transform.translation(0.0, 0.0, 1.0)
      ...> ]
      iex> t = BB.Robot.Transform.compose_all(transforms)
      iex> BB.Robot.Transform.get_translation(t)
      {1.0, 1.0, 1.0}
  """
  @spec compose_all([Nx.Tensor.t()]) :: Nx.Tensor.t()
  def compose_all([]), do: identity()
  def compose_all([t]), do: t
  def compose_all([h | t]), do: Enum.reduce(t, h, &compose(&2, &1))

  @doc """
  Get the translation component of a transform as {x, y, z}.
  """
  @spec get_translation(Nx.Tensor.t()) :: {float(), float(), float()}
  def get_translation(transform) do
    x = transform[0][3] |> Nx.to_number()
    y = transform[1][3] |> Nx.to_number()
    z = transform[2][3] |> Nx.to_number()
    {x, y, z}
  end

  @doc """
  Get the rotation matrix (3x3) from a transform.
  """
  @spec get_rotation(Nx.Tensor.t()) :: Nx.Tensor.t()
  def get_rotation(transform) do
    transform[0..2][0..2]
  end

  @doc """
  Apply a transform to a 3D point, returning the transformed point.

  ## Examples

      iex> t = BB.Robot.Transform.translation(1.0, 2.0, 3.0)
      iex> BB.Robot.Transform.apply_to_point(t, {0.0, 0.0, 0.0})
      {1.0, 2.0, 3.0}
  """
  @spec apply_to_point(Nx.Tensor.t(), {float(), float(), float()}) ::
          {float(), float(), float()}
  def apply_to_point(transform, {x, y, z}) do
    point = Nx.tensor([x, y, z, 1.0], type: :f64)
    result = Nx.dot(transform, point)

    {
      Nx.to_number(result[0]),
      Nx.to_number(result[1]),
      Nx.to_number(result[2])
    }
  end

  @doc """
  Compute the inverse of a transformation matrix.

  For a valid transformation matrix, this computes the inverse transform.
  """
  @spec inverse(Nx.Tensor.t()) :: Nx.Tensor.t()
  def inverse(transform) do
    r = get_rotation(transform)
    {tx, ty, tz} = get_translation(transform)

    r_inv = Nx.transpose(r)

    t_vec = Nx.tensor([tx, ty, tz], type: :f64)
    t_inv = Nx.negate(Nx.dot(r_inv, t_vec))

    Nx.tensor(
      [
        [
          Nx.to_number(r_inv[0][0]),
          Nx.to_number(r_inv[0][1]),
          Nx.to_number(r_inv[0][2]),
          Nx.to_number(t_inv[0])
        ],
        [
          Nx.to_number(r_inv[1][0]),
          Nx.to_number(r_inv[1][1]),
          Nx.to_number(r_inv[1][2]),
          Nx.to_number(t_inv[1])
        ],
        [
          Nx.to_number(r_inv[2][0]),
          Nx.to_number(r_inv[2][1]),
          Nx.to_number(r_inv[2][2]),
          Nx.to_number(t_inv[2])
        ],
        [0.0, 0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  @doc """
  Create a transform for a revolute joint rotation around an axis.

  ## Parameters

  - `axis`: normalised axis vector {x, y, z}
  - `angle`: rotation angle in radians

  ## Examples

      iex> axis = {0.0, 0.0, 1.0}  # Z axis
      iex> t = BB.Robot.Transform.revolute_transform(axis, :math.pi() / 2)
      iex> {x, y, _z} = BB.Robot.Transform.apply_to_point(t, {1.0, 0.0, 0.0})
      iex> {Float.round(x, 6), Float.round(y, 6)}
      {0.0, 1.0}
  """
  @spec revolute_transform({float(), float(), float()}, float()) :: Nx.Tensor.t()
  def revolute_transform({ax, ay, az}, angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)
    t = 1.0 - c

    # Rodrigues rotation formula - standard right-hand rule convention
    # +90° around Z takes X → +Y, +90° around Y takes X → -Z
    Nx.tensor(
      [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
        [0.0, 0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  @doc """
  Create a transform for a prismatic joint translation along an axis.

  ## Parameters

  - `axis`: normalised axis vector {x, y, z}
  - `distance`: translation distance in meters

  ## Examples

      iex> axis = {1.0, 0.0, 0.0}  # X axis
      iex> t = BB.Robot.Transform.prismatic_transform(axis, 2.5)
      iex> BB.Robot.Transform.get_translation(t)
      {2.5, 0.0, 0.0}
  """
  @spec prismatic_transform({float(), float(), float()}, float()) :: Nx.Tensor.t()
  def prismatic_transform({ax, ay, az}, distance) do
    translation(ax * distance, ay * distance, az * distance)
  end
end
