# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Quaternion do
  @moduledoc """
  Unit quaternion for 3D rotations, held as four `:f64` floats.

  Components are named for WXYZ order (scalar first), and every operation
  returns a normalised unit quaternion suitable for representing a rotation.

  ## Not tensor-backed

  Like `BB.Math.Vec3` and `BB.Math.Transform2D`, a single quaternion is far
  cheaper as four BEAM floats than as an `Nx` tensor. A Hamilton product is
  sixteen multiplies and twelve adds; dispatching those through eager `Nx` ops -
  or through a `defn`, which retraces its expression graph per call - costs
  orders of magnitude more than the arithmetic, and allocates every
  intermediate. Orientation is built and read on every estimator sample, so that
  lands squarely in the hot path. `bb_estimator_ahrs` used to carry its own
  scalar quaternion for exactly this reason.

  `tensor/1` and `from_tensor/1` convert at the boundary of code that genuinely
  wants tensors, as do `to_rotation_matrix/1` and `from_rotation_matrix/1`.

  ## Examples

      iex> BB.Math.Quaternion.new(1, 0, 0, 0)
      BB.Math.Quaternion.new(1.0, 0.0, 0.0, 0.0)

      iex> q = BB.Math.Quaternion.identity()
      iex> BB.Math.Quaternion.w(q)
      1.0

      iex> q1 = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> q2 = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> q3 = BB.Math.Quaternion.multiply(q1, q2)
      iex> BB.Math.Quaternion.angular_distance(q3, BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi()))
      0.0
  """

  alias BB.Math.Vec3

  defstruct [:w, :x, :y, :z]

  @type t :: %__MODULE__{w: float(), x: float(), y: float(), z: float()}

  @zero_threshold 1.0e-10
  @gimbal_threshold 0.99999
  @slerp_linear_threshold 0.9995
  @antiparallel_threshold -0.9999

  defimpl Inspect do
    import Inspect.Algebra

    @spec inspect(@for.t(), Inspect.Opts.t()) :: Inspect.Algebra.t()
    def inspect(%@for{w: w, x: x, y: y, z: z}, opts) do
      container_doc("BB.Math.Quaternion.new(", [w, x, y, z], ")", opts, &to_doc/2)
    end
  end

  @doc """
  Creates a new quaternion from w, x, y, z components.

  The quaternion is automatically normalised.

  ## Examples

      iex> q = BB.Math.Quaternion.new(1, 0, 0, 0)
      iex> BB.Math.Quaternion.w(q)
      1.0
  """
  @spec new(number(), number(), number(), number()) :: t()
  def new(w, x, y, z) do
    normalise(%__MODULE__{w: w / 1, x: x / 1, y: y / 1, z: z / 1})
  end

  @doc """
  Creates a quaternion from an existing `{4}` tensor.

  The tensor should be in WXYZ order. It will be normalised.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor) do
    [w, x, y, z] = tensor |> Nx.as_type(:f64) |> Nx.to_flat_list()

    new(w, x, y, z)
  end

  @doc """
  Returns the identity quaternion (no rotation).

  ## Examples

      iex> q = BB.Math.Quaternion.identity()
      iex> {BB.Math.Quaternion.w(q), BB.Math.Quaternion.x(q), BB.Math.Quaternion.y(q), BB.Math.Quaternion.z(q)}
      {1.0, 0.0, 0.0, 0.0}
  """
  @spec identity() :: t()
  def identity, do: %__MODULE__{w: 1.0, x: 0.0, y: 0.0, z: 0.0}

  @doc """
  Returns an identity quaternion as a raw tensor (for batch operations).
  """
  @spec identity_tensor() :: Nx.Tensor.t()
  def identity_tensor do
    Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64)
  end

  @doc "Returns the W (scalar) component."
  @spec w(t()) :: float()
  def w(%__MODULE__{w: w}), do: w

  @doc "Returns the X component."
  @spec x(t()) :: float()
  def x(%__MODULE__{x: x}), do: x

  @doc "Returns the Y component."
  @spec y(t()) :: float()
  def y(%__MODULE__{y: y}), do: y

  @doc "Returns the Z component."
  @spec z(t()) :: float()
  def z(%__MODULE__{z: z}), do: z

  @doc "Returns the quaternion as a `{4}` `:f64` WXYZ tensor."
  @spec tensor(t()) :: Nx.Tensor.t()
  def tensor(%__MODULE__{w: w, x: x, y: y, z: z}), do: Nx.tensor([w, x, y, z], type: :f64)

  @doc """
  Creates a quaternion from an axis-angle representation.

  The angle is in radians.

  ## Examples

      iex> q = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> Float.round(BB.Math.Quaternion.w(q), 6)
      0.707107
  """
  @spec from_axis_angle(Vec3.t(), number()) :: t()
  def from_axis_angle(%Vec3{} = axis, angle) do
    {ax, ay, az} =
      case Vec3.magnitude(axis) do
        norm when norm > @zero_threshold -> {axis.x / norm, axis.y / norm, axis.z / norm}
        _zero -> {0.0, 0.0, 1.0}
      end

    half = angle / 2
    sin_half = :math.sin(half)

    new(:math.cos(half), ax * sin_half, ay * sin_half, az * sin_half)
  end

  @doc """
  Creates a quaternion representing the shortest rotation from one vector to another.

  Both vectors should be unit vectors (they will be normalised if not).
  Returns the quaternion that rotates `from` to align with `to`.

  Handles edge cases:
  - Parallel vectors (from ≈ to): returns identity quaternion
  - Anti-parallel vectors (from ≈ -to): returns 180° rotation around a perpendicular axis

  ## Examples

      iex> q = BB.Math.Quaternion.from_two_vectors(BB.Math.Vec3.unit_x(), BB.Math.Vec3.unit_y())
      iex> rotated = BB.Math.Quaternion.rotate_vector(q, BB.Math.Vec3.unit_x())
      iex> {Float.round(BB.Math.Vec3.x(rotated), 6), Float.round(BB.Math.Vec3.y(rotated), 6)}
      {0.0, 1.0}

      iex> q = BB.Math.Quaternion.from_two_vectors(BB.Math.Vec3.unit_z(), BB.Math.Vec3.unit_z())
      iex> BB.Math.Quaternion.w(q)
      1.0
  """
  @spec from_two_vectors(Vec3.t(), Vec3.t()) :: t()
  def from_two_vectors(%Vec3{} = from, %Vec3{} = to) do
    from_unit = unit_or_x(from)
    to_unit = unit_or_x(to)

    dot = Vec3.dot(from_unit, to_unit)

    if dot < @antiparallel_threshold do
      perpendicular = perpendicular_to(from_unit)

      new(0.0, perpendicular.x, perpendicular.y, perpendicular.z)
    else
      cross = Vec3.cross(from_unit, to_unit)

      new(1.0 + dot, cross.x, cross.y, cross.z)
    end
  end

  defp unit_or_x(%Vec3{} = v) do
    case Vec3.magnitude(v) do
      magnitude when magnitude > @zero_threshold -> Vec3.scale(v, 1 / magnitude)
      _zero -> Vec3.unit_x()
    end
  end

  # The basis vector least aligned with `v` gives the numerically strongest
  # perpendicular, so cross against that one rather than a fixed axis.
  defp perpendicular_to(%Vec3{x: x, y: y, z: z} = v) do
    {ax, ay, az} = {abs(x), abs(y), abs(z)}

    basis =
      cond do
        ax <= ay and ax <= az -> Vec3.unit_x()
        ay <= az -> Vec3.unit_y()
        true -> Vec3.unit_z()
      end

    cross = Vec3.cross(v, basis)

    if Vec3.magnitude(cross) > @zero_threshold do
      Vec3.normalise(cross)
    else
      Vec3.unit_y()
    end
  end

  @doc """
  Creates a quaternion from a 3x3 rotation matrix.

  Accepts a `{3, 3}` tensor or a list of three three-element rows. Uses the
  Shepperd method for numerical stability: the branch is chosen so the value
  under the square root is largest, which keeps the division well conditioned.

  ## Examples

      iex> m = Nx.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
      iex> q = BB.Math.Quaternion.from_rotation_matrix(m)
      iex> BB.Math.Quaternion.w(q)
      1.0
  """
  @spec from_rotation_matrix(Nx.Tensor.t() | [[number()]]) :: t()
  def from_rotation_matrix(%Nx.Tensor{} = matrix) do
    matrix
    |> Nx.as_type(:f64)
    |> Nx.to_flat_list()
    |> from_rotation_elements()
  end

  def from_rotation_matrix([[_, _, _], [_, _, _], [_, _, _]] = rows) do
    rows
    |> List.flatten()
    |> Enum.map(&(&1 / 1))
    |> from_rotation_elements()
  end

  defp from_rotation_elements([m00, m01, m02, m10, m11, m12, m20, m21, m22]) do
    trace = m00 + m11 + m22

    cond do
      trace > 0 ->
        s = :math.sqrt(trace + 1.0) * 2
        new(s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)

      m00 > m11 and m00 > m22 ->
        s = :math.sqrt(1.0 + m00 - m11 - m22) * 2
        new((m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s)

      m11 > m22 ->
        s = :math.sqrt(1.0 - m00 + m11 - m22) * 2
        new((m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s)

      true ->
        s = :math.sqrt(1.0 - m00 - m11 + m22) * 2
        new((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4)
    end
  end

  @doc """
  Creates a quaternion from Euler angles (roll, pitch, yaw).

  Angles are in radians. Default order is `:xyz` (roll around X, pitch around Y, yaw around Z).

  Supported orders: `:xyz`, `:zyx`

  ## Examples

      iex> q = BB.Math.Quaternion.from_euler(0, 0, :math.pi() / 2, :xyz)
      iex> Float.round(BB.Math.Quaternion.z(q), 6)
      0.707107
  """
  @spec from_euler(number(), number(), number(), atom()) :: t()
  def from_euler(roll, pitch, yaw, order \\ :xyz) do
    c1 = :math.cos(roll / 2)
    c2 = :math.cos(pitch / 2)
    c3 = :math.cos(yaw / 2)
    s1 = :math.sin(roll / 2)
    s2 = :math.sin(pitch / 2)
    s3 = :math.sin(yaw / 2)

    euler_to_quaternion(order, c1, c2, c3, s1, s2, s3)
  end

  defp euler_to_quaternion(:zyx, c1, c2, c3, s1, s2, s3) do
    new(
      c1 * c2 * c3 + s1 * s2 * s3,
      s1 * c2 * c3 - c1 * s2 * s3,
      c1 * s2 * c3 + s1 * c2 * s3,
      c1 * c2 * s3 - s1 * s2 * c3
    )
  end

  defp euler_to_quaternion(_xyz, c1, c2, c3, s1, s2, s3) do
    new(
      c1 * c2 * c3 - s1 * s2 * s3,
      s1 * c2 * c3 + c1 * s2 * s3,
      c1 * s2 * c3 - s1 * c2 * s3,
      c1 * c2 * s3 + s1 * s2 * c3
    )
  end

  @doc """
  Converts a quaternion to a 3x3 rotation matrix as a `{3, 3}` `:f64` tensor.

  Use `to_rotation_list/1` to avoid building a tensor.

  ## Examples

      iex> q = BB.Math.Quaternion.identity()
      iex> m = BB.Math.Quaternion.to_rotation_matrix(q)
      iex> Nx.to_number(m[0][0])
      1.0
  """
  @spec to_rotation_matrix(t()) :: Nx.Tensor.t()
  def to_rotation_matrix(%__MODULE__{} = q) do
    Nx.tensor(to_rotation_list(q), type: :f64)
  end

  @doc """
  Converts a quaternion to a 3x3 rotation matrix as a list of rows.

  ## Examples

      iex> BB.Math.Quaternion.to_rotation_list(BB.Math.Quaternion.identity())
      [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  """
  @spec to_rotation_list(t()) :: [[float()]]
  def to_rotation_list(%__MODULE__{w: w, x: x, y: y, z: z}) do
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    [
      [1.0 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
      [2 * (xy + wz), 1.0 - 2 * (xx + zz), 2 * (yz - wx)],
      [2 * (xz - wy), 2 * (yz + wx), 1.0 - 2 * (xx + yy)]
    ]
  end

  @doc """
  Converts a quaternion to axis-angle representation.

  Returns `{axis, angle}` where axis is a `BB.Math.Vec3` unit vector
  and angle is in radians (0 to pi).

  ## Examples

      iex> q = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> {axis, angle} = BB.Math.Quaternion.to_axis_angle(q)
      iex> Float.round(angle, 6)
      1.570796
      iex> Float.round(BB.Math.Vec3.z(axis), 1)
      1.0
  """
  @spec to_axis_angle(t()) :: {Vec3.t(), float()}
  def to_axis_angle(%__MODULE__{w: w, x: x, y: y, z: z}) do
    angle = 2 * :math.acos(clamp(w, -1.0, 1.0))
    sin_half = :math.sin(angle / 2)

    if abs(sin_half) < @zero_threshold do
      {Vec3.unit_z(), angle}
    else
      {Vec3.new(x / sin_half, y / sin_half, z / sin_half), angle}
    end
  end

  @doc """
  Converts a quaternion to Euler angles (roll, pitch, yaw).

  Returns `{roll, pitch, yaw}` in radians. Default order is `:xyz`.

  Note: Euler angles can have gimbal lock issues near pitch = ±90°.

  ## Examples

      iex> q = BB.Math.Quaternion.from_euler(0.1, 0.2, 0.3, :xyz)
      iex> {roll, pitch, yaw} = BB.Math.Quaternion.to_euler(q, :xyz)
      iex> Float.round(roll, 6)
      0.1
  """
  @spec to_euler(t(), atom()) :: {float(), float(), float()}
  def to_euler(%__MODULE__{} = q, order \\ :xyz) do
    rotation_to_euler(order, to_rotation_list(q))
  end

  # ZYX order (yaw-pitch-roll, common in aerospace)
  defp rotation_to_euler(:zyx, [[m00, _m01, _m02], [m10, _m11, _m12], [m20, m21, m22]]) do
    cond do
      m20 <= -@gimbal_threshold -> {0.0, :math.pi() / 2, :math.atan2(-m10, m00)}
      m20 >= @gimbal_threshold -> {0.0, -:math.pi() / 2, :math.atan2(-m10, m00)}
      true -> {:math.atan2(m21, m22), :math.asin(clamp(-m20, -1.0, 1.0)), :math.atan2(m10, m00)}
    end
  end

  # XYZ order (roll-pitch-yaw). For intrinsic XYZ: R = Rx(roll) * Ry(pitch) * Rz(yaw)
  defp rotation_to_euler(_xyz, [[m00, m01, m02], [m10, m11, m12], [_m20, _m21, m22]]) do
    cond do
      m02 >= @gimbal_threshold -> {0.0, :math.pi() / 2, :math.atan2(m10, m11)}
      m02 <= -@gimbal_threshold -> {0.0, -:math.pi() / 2, :math.atan2(m10, m11)}
      true -> {:math.atan2(-m12, m22), :math.asin(clamp(m02, -1.0, 1.0)), :math.atan2(-m01, m00)}
    end
  end

  @doc """
  Multiplies two quaternions (Hamilton product).

  This composes the rotations: `multiply(q1, q2)` applies q2 first, then q1.

  ## Examples

      iex> q1 = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> q2 = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> q3 = BB.Math.Quaternion.multiply(q1, q2)
      iex> {_axis, angle} = BB.Math.Quaternion.to_axis_angle(q3)
      iex> Float.round(angle, 6)
      3.141593
  """
  @spec multiply(t(), t()) :: t()
  def multiply(%__MODULE__{} = a, %__MODULE__{} = b) do
    new(
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    )
  end

  @doc """
  Returns the conjugate of a quaternion.

  For unit quaternions, the conjugate equals the inverse.

  ## Examples

      iex> q = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> qc = BB.Math.Quaternion.conjugate(q)
      iex> Float.round(BB.Math.Quaternion.z(qc), 6)
      -0.707107
  """
  @spec conjugate(t()) :: t()
  def conjugate(%__MODULE__{w: w, x: x, y: y, z: z}) do
    %__MODULE__{w: w, x: -x, y: -y, z: -z}
  end

  @doc """
  Normalises a quaternion to unit length.

  Falls back to the identity quaternion when the input is near-zero, so the
  result is always a valid unit rotation.

  ## Examples

      iex> q = BB.Math.Quaternion.normalise(%BB.Math.Quaternion{w: 2.0, x: 0.0, y: 0.0, z: 0.0})
      iex> BB.Math.Quaternion.w(q)
      1.0
  """
  @spec normalise(t()) :: t()
  def normalise(%__MODULE__{w: w, x: x, y: y, z: z}) do
    case :math.sqrt(w * w + x * x + y * y + z * z) do
      norm when norm > @zero_threshold ->
        %__MODULE__{w: w / norm, x: x / norm, y: y / norm, z: z / norm}

      _zero ->
        identity()
    end
  end

  @doc """
  Returns the inverse of a quaternion.

  For unit quaternions, this equals the conjugate.

  ## Examples

      iex> q = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> qi = BB.Math.Quaternion.inverse(q)
      iex> qr = BB.Math.Quaternion.multiply(q, qi)
      iex> Float.round(BB.Math.Quaternion.w(qr), 6)
      1.0
  """
  @spec inverse(t()) :: t()
  def inverse(%__MODULE__{} = q), do: conjugate(q)

  @doc """
  Rotates a 3D vector by a quaternion.

  ## Examples

      iex> q = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> v = BB.Math.Vec3.unit_x()
      iex> rotated = BB.Math.Quaternion.rotate_vector(q, v)
      iex> {Float.round(BB.Math.Vec3.x(rotated), 6), Float.round(BB.Math.Vec3.y(rotated), 6)}
      {0.0, 1.0}
  """
  @spec rotate_vector(t(), Vec3.t()) :: Vec3.t()
  def rotate_vector(%__MODULE__{w: w, x: x, y: y, z: z}, %Vec3{} = v) do
    # Rodrigues: v' = v + 2w(u × v) + 2(u × (u × v)), with u the vector part.
    u = %Vec3{x: x, y: y, z: z}
    uxv = Vec3.cross(u, v)
    uuxv = Vec3.cross(u, uxv)

    %Vec3{
      x: v.x + 2 * w * uxv.x + 2 * uuxv.x,
      y: v.y + 2 * w * uxv.y + 2 * uuxv.y,
      z: v.z + 2 * w * uxv.z + 2 * uuxv.z
    }
  end

  @doc """
  Spherical linear interpolation between two quaternions.

  `t` should be between 0.0 and 1.0.

  ## Examples

      iex> q1 = BB.Math.Quaternion.identity()
      iex> q2 = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi())
      iex> q_mid = BB.Math.Quaternion.slerp(q1, q2, 0.5)
      iex> {_axis, angle} = BB.Math.Quaternion.to_axis_angle(q_mid)
      iex> Float.round(angle, 6)
      1.570796
  """
  @spec slerp(t(), t(), number()) :: t()
  def slerp(%__MODULE__{} = a, %__MODULE__{} = b, t) when t >= 0 and t <= 1 do
    dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z

    # q and -q are the same rotation, so flip to take the shorter arc.
    {b, dot} = if dot < 0, do: {negate(b), -dot}, else: {b, dot}

    {s1, s2} =
      case clamp(dot, 0.0, 1.0) do
        close when close > @slerp_linear_threshold ->
          {1 - t, t}

        clamped ->
          theta = :math.acos(clamped)
          sin_theta = :math.sin(theta)

          {:math.sin((1 - t) * theta) / sin_theta, :math.sin(t * theta) / sin_theta}
      end

    new(
      a.w * s1 + b.w * s2,
      a.x * s1 + b.x * s2,
      a.y * s1 + b.y * s2,
      a.z * s1 + b.z * s2
    )
  end

  defp negate(%__MODULE__{w: w, x: x, y: y, z: z}) do
    %__MODULE__{w: -w, x: -x, y: -y, z: -z}
  end

  @doc """
  Computes the angular distance between two quaternions in radians.

  Returns a value between 0 and pi.

  ## Examples

      iex> q1 = BB.Math.Quaternion.identity()
      iex> q2 = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 2)
      iex> Float.round(BB.Math.Quaternion.angular_distance(q1, q2), 6)
      1.570796
  """
  @spec angular_distance(t(), t()) :: float()
  def angular_distance(%__MODULE__{} = a, %__MODULE__{} = b) do
    dot = abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z)

    2 * :math.acos(clamp(dot, 0.0, 1.0))
  end

  @doc """
  Converts to a list in XYZW order (for ROS/external system compatibility).

  ## Examples

      iex> q = BB.Math.Quaternion.identity()
      iex> BB.Math.Quaternion.to_xyzw_list(q)
      [0.0, 0.0, 0.0, 1.0]
  """
  @spec to_xyzw_list(t()) :: [float()]
  def to_xyzw_list(%__MODULE__{w: w, x: x, y: y, z: z}), do: [x, y, z, w]

  @doc """
  Creates from a list in XYZW order (for ROS/external system compatibility).

  ## Examples

      iex> q = BB.Math.Quaternion.from_xyzw_list([0.0, 0.0, 0.0, 1.0])
      iex> BB.Math.Quaternion.w(q)
      1.0
  """
  @spec from_xyzw_list([number()]) :: t()
  def from_xyzw_list([x, y, z, w]), do: new(w, x, y, z)

  @doc """
  Converts to a list in WXYZ order.

  ## Examples

      iex> q = BB.Math.Quaternion.identity()
      iex> BB.Math.Quaternion.to_list(q)
      [1.0, 0.0, 0.0, 0.0]
  """
  @spec to_list(t()) :: [float()]
  def to_list(%__MODULE__{w: w, x: x, y: y, z: z}), do: [w, x, y, z]

  @doc """
  Creates from a list in WXYZ order.

  ## Examples

      iex> q = BB.Math.Quaternion.from_list([1.0, 0.0, 0.0, 0.0])
      iex> BB.Math.Quaternion.w(q)
      1.0
  """
  @spec from_list([number()]) :: t()
  def from_list([w, x, y, z]), do: new(w, x, y, z)

  defp clamp(value, min, max), do: value |> max(min) |> min(max)
end
