# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Quaternion do
  @moduledoc """
  Unit quaternion for 3D rotations, backed by an Nx tensor.

  Quaternions are stored in WXYZ order (scalar first): `[w, x, y, z]`.
  All math operations use Nx for consistent performance and potential GPU acceleration.

  All operations return normalised unit quaternions suitable for representing rotations.
  The underlying tensor is always `{4}` shape with `:f64` type.

  ## Examples

      iex> q = BB.Quaternion.identity()
      iex> BB.Quaternion.w(q)
      1.0

      iex> q1 = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> q2 = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> q3 = BB.Quaternion.multiply(q1, q2)
      iex> BB.Quaternion.angular_distance(q3, BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi()))
      0.0
  """

  alias BB.Vec3

  defstruct [:tensor]

  @type t :: %__MODULE__{tensor: Nx.Tensor.t()}

  @doc """
  Creates a new quaternion from w, x, y, z components.

  The quaternion is automatically normalised.

  ## Examples

      iex> q = BB.Quaternion.new(1, 0, 0, 0)
      iex> BB.Quaternion.w(q)
      1.0
  """
  @spec new(number(), number(), number(), number()) :: t()
  def new(w, x, y, z) do
    tensor = Nx.tensor([w, x, y, z], type: :f64)
    %__MODULE__{tensor: normalise_tensor(tensor)}
  end

  @doc """
  Creates a quaternion from an existing `{4}` tensor.

  The tensor should be in WXYZ order. It will be normalised.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor) do
    %__MODULE__{tensor: normalise_tensor(Nx.as_type(tensor, :f64))}
  end

  @doc """
  Returns the identity quaternion (no rotation).

  ## Examples

      iex> q = BB.Quaternion.identity()
      iex> {BB.Quaternion.w(q), BB.Quaternion.x(q), BB.Quaternion.y(q), BB.Quaternion.z(q)}
      {1.0, 0.0, 0.0, 0.0}
  """
  @spec identity() :: t()
  def identity do
    %__MODULE__{tensor: Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64)}
  end

  @doc """
  Returns an identity quaternion as a raw tensor (for batch operations).
  """
  @spec identity_tensor() :: Nx.Tensor.t()
  def identity_tensor do
    Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64)
  end

  # Component accessors

  @doc "Returns the W (scalar) component."
  @spec w(t()) :: float()
  def w(%__MODULE__{tensor: t}), do: Nx.to_number(t[0])

  @doc "Returns the X component."
  @spec x(t()) :: float()
  def x(%__MODULE__{tensor: t}), do: Nx.to_number(t[1])

  @doc "Returns the Y component."
  @spec y(t()) :: float()
  def y(%__MODULE__{tensor: t}), do: Nx.to_number(t[2])

  @doc "Returns the Z component."
  @spec z(t()) :: float()
  def z(%__MODULE__{tensor: t}), do: Nx.to_number(t[3])

  @doc "Returns the underlying `{4}` tensor."
  @spec tensor(t()) :: Nx.Tensor.t()
  def tensor(%__MODULE__{tensor: t}), do: t

  @doc """
  Creates a quaternion from an axis-angle representation.

  The axis should be a `BB.Vec3` unit vector (it will be normalised if not).
  The angle is in radians.

  ## Examples

      iex> q = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> Float.round(BB.Quaternion.w(q), 6)
      0.707107
  """
  @spec from_axis_angle(Vec3.t(), number()) :: t()
  def from_axis_angle(%Vec3{tensor: axis_tensor}, angle) do
    # Normalise axis
    axis_norm = Nx.LinAlg.norm(axis_tensor)
    default_axis = Nx.tensor([0.0, 0.0, 1.0], type: :f64)

    axis_normalised =
      Nx.select(
        Nx.greater(axis_norm, 1.0e-10),
        Nx.divide(axis_tensor, axis_norm),
        default_axis
      )

    half_angle = Nx.tensor(angle / 2, type: :f64)
    sin_half = Nx.sin(half_angle)
    cos_half = Nx.cos(half_angle)

    xyz = Nx.multiply(axis_normalised, sin_half)
    tensor = Nx.concatenate([Nx.reshape(cos_half, {1}), xyz])

    %__MODULE__{tensor: tensor}
  end

  @doc """
  Creates a quaternion from a 3x3 rotation matrix.

  Uses the Shepperd method for numerical stability.

  ## Examples

      iex> m = Nx.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
      iex> q = BB.Quaternion.from_rotation_matrix(m)
      iex> BB.Quaternion.w(q)
      1.0
  """
  @spec from_rotation_matrix(Nx.Tensor.t()) :: t()
  def from_rotation_matrix(matrix) do
    matrix = Nx.as_type(matrix, :f64)

    # Extract matrix elements
    m00 = matrix[0][0]
    m01 = matrix[0][1]
    m02 = matrix[0][2]
    m10 = matrix[1][0]
    m11 = matrix[1][1]
    m12 = matrix[1][2]
    m20 = matrix[2][0]
    m21 = matrix[2][1]
    m22 = matrix[2][2]

    trace = Nx.add(Nx.add(m00, m11), m22)

    # Compute all 4 cases and select the best one
    # Case 0: trace > 0
    s0 = Nx.multiply(Nx.sqrt(Nx.add(trace, 1.0)), 2.0)
    w0 = Nx.divide(s0, 4.0)
    x0 = Nx.divide(Nx.subtract(m21, m12), s0)
    y0 = Nx.divide(Nx.subtract(m02, m20), s0)
    z0 = Nx.divide(Nx.subtract(m10, m01), s0)
    q0 = Nx.stack([w0, x0, y0, z0])

    # Case 1: m00 is largest diagonal
    s1 = Nx.multiply(Nx.sqrt(Nx.add(Nx.subtract(Nx.subtract(1.0, m11), m22), m00)), 2.0)
    w1 = Nx.divide(Nx.subtract(m21, m12), s1)
    x1 = Nx.divide(s1, 4.0)
    y1 = Nx.divide(Nx.add(m01, m10), s1)
    z1 = Nx.divide(Nx.add(m02, m20), s1)
    q1 = Nx.stack([w1, x1, y1, z1])

    # Case 2: m11 is largest diagonal
    s2 = Nx.multiply(Nx.sqrt(Nx.add(Nx.subtract(Nx.subtract(1.0, m00), m22), m11)), 2.0)
    w2 = Nx.divide(Nx.subtract(m02, m20), s2)
    x2 = Nx.divide(Nx.add(m01, m10), s2)
    y2 = Nx.divide(s2, 4.0)
    z2 = Nx.divide(Nx.add(m12, m21), s2)
    q2 = Nx.stack([w2, x2, y2, z2])

    # Case 3: m22 is largest diagonal
    s3 = Nx.multiply(Nx.sqrt(Nx.add(Nx.subtract(Nx.subtract(1.0, m00), m11), m22)), 2.0)
    w3 = Nx.divide(Nx.subtract(m10, m01), s3)
    x3 = Nx.divide(Nx.add(m02, m20), s3)
    y3 = Nx.divide(Nx.add(m12, m21), s3)
    z3 = Nx.divide(s3, 4.0)
    q3 = Nx.stack([w3, x3, y3, z3])

    # Select based on which case applies
    # trace > 0 -> case 0
    # else m00 > m11 and m00 > m22 -> case 1
    # else m11 > m22 -> case 2
    # else -> case 3
    result =
      Nx.select(
        Nx.greater(trace, 0),
        q0,
        Nx.select(
          Nx.logical_and(Nx.greater(m00, m11), Nx.greater(m00, m22)),
          q1,
          Nx.select(
            Nx.greater(m11, m22),
            q2,
            q3
          )
        )
      )

    %__MODULE__{tensor: normalise_tensor(result)}
  end

  @doc """
  Creates a quaternion from Euler angles (roll, pitch, yaw).

  Angles are in radians. Default order is `:xyz` (roll around X, pitch around Y, yaw around Z).

  Supported orders: `:xyz`, `:zyx`

  ## Examples

      iex> q = BB.Quaternion.from_euler(0, 0, :math.pi() / 2, :xyz)
      iex> Float.round(BB.Quaternion.z(q), 6)
      0.707107
  """
  @spec from_euler(number(), number(), number(), atom()) :: t()
  def from_euler(roll, pitch, yaw, order \\ :xyz) do
    # Half angles as tensors
    x2 = Nx.tensor(roll / 2, type: :f64)
    y2 = Nx.tensor(pitch / 2, type: :f64)
    z2 = Nx.tensor(yaw / 2, type: :f64)

    c1 = Nx.cos(x2)
    c2 = Nx.cos(y2)
    c3 = Nx.cos(z2)
    s1 = Nx.sin(x2)
    s2 = Nx.sin(y2)
    s3 = Nx.sin(z2)

    tensor = euler_to_quaternion_tensor(order, c1, c2, c3, s1, s2, s3)
    %__MODULE__{tensor: normalise_tensor(tensor)}
  end

  defp euler_to_quaternion_tensor(:xyz, c1, c2, c3, s1, s2, s3) do
    # x = s1 * c2 * c3 + c1 * s2 * s3
    x =
      Nx.add(
        Nx.multiply(Nx.multiply(s1, c2), c3),
        Nx.multiply(Nx.multiply(c1, s2), s3)
      )

    # y = c1 * s2 * c3 - s1 * c2 * s3
    y =
      Nx.subtract(
        Nx.multiply(Nx.multiply(c1, s2), c3),
        Nx.multiply(Nx.multiply(s1, c2), s3)
      )

    # z = c1 * c2 * s3 + s1 * s2 * c3
    z =
      Nx.add(
        Nx.multiply(Nx.multiply(c1, c2), s3),
        Nx.multiply(Nx.multiply(s1, s2), c3)
      )

    # w = c1 * c2 * c3 - s1 * s2 * s3
    w =
      Nx.subtract(
        Nx.multiply(Nx.multiply(c1, c2), c3),
        Nx.multiply(Nx.multiply(s1, s2), s3)
      )

    Nx.stack([w, x, y, z])
  end

  defp euler_to_quaternion_tensor(:zyx, c1, c2, c3, s1, s2, s3) do
    # x = s1 * c2 * c3 - c1 * s2 * s3
    x =
      Nx.subtract(
        Nx.multiply(Nx.multiply(s1, c2), c3),
        Nx.multiply(Nx.multiply(c1, s2), s3)
      )

    # y = c1 * s2 * c3 + s1 * c2 * s3
    y =
      Nx.add(
        Nx.multiply(Nx.multiply(c1, s2), c3),
        Nx.multiply(Nx.multiply(s1, c2), s3)
      )

    # z = c1 * c2 * s3 - s1 * s2 * c3
    z =
      Nx.subtract(
        Nx.multiply(Nx.multiply(c1, c2), s3),
        Nx.multiply(Nx.multiply(s1, s2), c3)
      )

    # w = c1 * c2 * c3 + s1 * s2 * s3
    w =
      Nx.add(
        Nx.multiply(Nx.multiply(c1, c2), c3),
        Nx.multiply(Nx.multiply(s1, s2), s3)
      )

    Nx.stack([w, x, y, z])
  end

  # Default to xyz for unsupported orders
  defp euler_to_quaternion_tensor(_order, c1, c2, c3, s1, s2, s3) do
    euler_to_quaternion_tensor(:xyz, c1, c2, c3, s1, s2, s3)
  end

  @doc """
  Converts a quaternion to a 3x3 rotation matrix.

  ## Examples

      iex> q = BB.Quaternion.identity()
      iex> m = BB.Quaternion.to_rotation_matrix(q)
      iex> Nx.to_number(m[0][0])
      1.0
  """
  @spec to_rotation_matrix(t()) :: Nx.Tensor.t()
  def to_rotation_matrix(%__MODULE__{tensor: t}) do
    w = t[0]
    x = t[1]
    y = t[2]
    z = t[3]

    # Pre-compute products
    xx = Nx.multiply(x, x)
    yy = Nx.multiply(y, y)
    zz = Nx.multiply(z, z)
    xy = Nx.multiply(x, y)
    xz = Nx.multiply(x, z)
    yz = Nx.multiply(y, z)
    wx = Nx.multiply(w, x)
    wy = Nx.multiply(w, y)
    wz = Nx.multiply(w, z)

    two = Nx.tensor(2.0, type: :f64)
    one = Nx.tensor(1.0, type: :f64)

    # Build rotation matrix
    r00 = Nx.subtract(one, Nx.multiply(two, Nx.add(yy, zz)))
    r01 = Nx.multiply(two, Nx.subtract(xy, wz))
    r02 = Nx.multiply(two, Nx.add(xz, wy))

    r10 = Nx.multiply(two, Nx.add(xy, wz))
    r11 = Nx.subtract(one, Nx.multiply(two, Nx.add(xx, zz)))
    r12 = Nx.multiply(two, Nx.subtract(yz, wx))

    r20 = Nx.multiply(two, Nx.subtract(xz, wy))
    r21 = Nx.multiply(two, Nx.add(yz, wx))
    r22 = Nx.subtract(one, Nx.multiply(two, Nx.add(xx, yy)))

    Nx.stack([
      Nx.stack([r00, r01, r02]),
      Nx.stack([r10, r11, r12]),
      Nx.stack([r20, r21, r22])
    ])
  end

  @doc """
  Converts a quaternion to axis-angle representation.

  Returns `{axis, angle}` where axis is a `BB.Vec3` unit vector
  and angle is in radians (0 to pi).

  ## Examples

      iex> q = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> {axis, angle} = BB.Quaternion.to_axis_angle(q)
      iex> Float.round(angle, 6)
      1.570796
      iex> Float.round(BB.Vec3.z(axis), 1)
      1.0
  """
  @spec to_axis_angle(t()) :: {Vec3.t(), float()}
  def to_axis_angle(%__MODULE__{tensor: t}) do
    w = t[0]
    xyz = Nx.slice(t, [1], [3])

    # Clamp w to valid range for acos
    w_clamped = Nx.clip(w, -1.0, 1.0)
    angle = Nx.multiply(2.0, Nx.acos(w_clamped))
    angle_float = Nx.to_number(angle)

    sin_half = Nx.sin(Nx.divide(angle, 2.0))

    # If sin_half is near zero, return arbitrary axis
    default_axis = Nx.tensor([0.0, 0.0, 1.0], type: :f64)

    axis_tensor =
      Nx.select(
        Nx.less(Nx.abs(sin_half), 1.0e-10),
        default_axis,
        Nx.divide(xyz, sin_half)
      )

    {Vec3.from_tensor(axis_tensor), angle_float}
  end

  @doc """
  Converts a quaternion to Euler angles (roll, pitch, yaw).

  Returns `{roll, pitch, yaw}` in radians. Default order is `:xyz`.

  Note: Euler angles can have gimbal lock issues near pitch = ±90°.

  ## Examples

      iex> q = BB.Quaternion.from_euler(0.1, 0.2, 0.3, :xyz)
      iex> {roll, pitch, yaw} = BB.Quaternion.to_euler(q, :xyz)
      iex> Float.round(roll, 6)
      0.1
  """
  @spec to_euler(t(), atom()) :: {float(), float(), float()}
  def to_euler(%__MODULE__{} = q, order \\ :xyz) do
    matrix = to_rotation_matrix(q)
    rotation_matrix_to_euler(order, matrix)
  end

  # XYZ order (roll-pitch-yaw)
  # For intrinsic XYZ: R = Rx(roll) * Ry(pitch) * Rz(yaw)
  defp rotation_matrix_to_euler(:xyz, matrix) do
    m02 = matrix[0][2]
    m12 = matrix[1][2]
    m22 = matrix[2][2]
    m01 = matrix[0][1]
    m00 = matrix[0][0]
    m10 = matrix[1][0]
    m11 = matrix[1][1]

    # Check for gimbal lock
    gimbal_pos = Nx.greater_equal(m02, 0.99999)
    gimbal_neg = Nx.less_equal(m02, -0.99999)

    # Normal case
    pitch_normal = Nx.asin(Nx.clip(m02, -1.0, 1.0))
    roll_normal = Nx.atan2(Nx.negate(m12), m22)
    yaw_normal = Nx.atan2(Nx.negate(m01), m00)

    # Gimbal lock case (pitch ≈ +90°)
    pitch_pos = Nx.tensor(:math.pi() / 2, type: :f64)
    roll_pos = Nx.tensor(0.0, type: :f64)
    yaw_pos = Nx.atan2(m10, m11)

    # Gimbal lock case (pitch ≈ -90°)
    pitch_neg = Nx.tensor(-:math.pi() / 2, type: :f64)
    roll_neg = Nx.tensor(0.0, type: :f64)
    yaw_neg = Nx.atan2(m10, m11)

    roll = Nx.select(gimbal_pos, roll_pos, Nx.select(gimbal_neg, roll_neg, roll_normal))
    pitch = Nx.select(gimbal_pos, pitch_pos, Nx.select(gimbal_neg, pitch_neg, pitch_normal))
    yaw = Nx.select(gimbal_pos, yaw_pos, Nx.select(gimbal_neg, yaw_neg, yaw_normal))

    {Nx.to_number(roll), Nx.to_number(pitch), Nx.to_number(yaw)}
  end

  # ZYX order (yaw-pitch-roll, common in aerospace)
  defp rotation_matrix_to_euler(:zyx, matrix) do
    m20 = matrix[2][0]
    m21 = matrix[2][1]
    m22 = matrix[2][2]
    m10 = matrix[1][0]
    m00 = matrix[0][0]

    gimbal_pos = Nx.less_equal(m20, -0.99999)
    gimbal_neg = Nx.greater_equal(m20, 0.99999)

    pitch_normal = Nx.asin(Nx.clip(Nx.negate(m20), -1.0, 1.0))
    roll_normal = Nx.atan2(m21, m22)
    yaw_normal = Nx.atan2(m10, m00)

    pitch_pos = Nx.tensor(:math.pi() / 2, type: :f64)
    roll_pos = Nx.tensor(0.0, type: :f64)
    yaw_pos = Nx.atan2(Nx.negate(m10), m00)

    pitch_neg = Nx.tensor(-:math.pi() / 2, type: :f64)
    roll_neg = Nx.tensor(0.0, type: :f64)
    yaw_neg = Nx.atan2(Nx.negate(m10), m00)

    roll = Nx.select(gimbal_pos, roll_pos, Nx.select(gimbal_neg, roll_neg, roll_normal))
    pitch = Nx.select(gimbal_pos, pitch_pos, Nx.select(gimbal_neg, pitch_neg, pitch_normal))
    yaw = Nx.select(gimbal_pos, yaw_pos, Nx.select(gimbal_neg, yaw_neg, yaw_normal))

    {Nx.to_number(roll), Nx.to_number(pitch), Nx.to_number(yaw)}
  end

  # Default to xyz for unsupported orders
  defp rotation_matrix_to_euler(_order, matrix) do
    rotation_matrix_to_euler(:xyz, matrix)
  end

  @doc """
  Multiplies two quaternions (Hamilton product).

  This composes the rotations: `multiply(q1, q2)` applies q2 first, then q1.

  ## Examples

      iex> q1 = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> q2 = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> q3 = BB.Quaternion.multiply(q1, q2)
      iex> {_axis, angle} = BB.Quaternion.to_axis_angle(q3)
      iex> Float.round(angle, 6)
      3.141593
  """
  @spec multiply(t(), t()) :: t()
  def multiply(%__MODULE__{tensor: t1}, %__MODULE__{tensor: t2}) do
    w1 = t1[0]
    x1 = t1[1]
    y1 = t1[2]
    z1 = t1[3]

    w2 = t2[0]
    x2 = t2[1]
    y2 = t2[2]
    z2 = t2[3]

    # Hamilton product
    # w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    w =
      Nx.subtract(
        Nx.subtract(
          Nx.subtract(
            Nx.multiply(w1, w2),
            Nx.multiply(x1, x2)
          ),
          Nx.multiply(y1, y2)
        ),
        Nx.multiply(z1, z2)
      )

    # x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    x =
      Nx.subtract(
        Nx.add(
          Nx.add(
            Nx.multiply(w1, x2),
            Nx.multiply(x1, w2)
          ),
          Nx.multiply(y1, z2)
        ),
        Nx.multiply(z1, y2)
      )

    # y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    y =
      Nx.add(
        Nx.add(
          Nx.subtract(
            Nx.multiply(w1, y2),
            Nx.multiply(x1, z2)
          ),
          Nx.multiply(y1, w2)
        ),
        Nx.multiply(z1, x2)
      )

    # z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    z =
      Nx.add(
        Nx.subtract(
          Nx.add(
            Nx.multiply(w1, z2),
            Nx.multiply(x1, y2)
          ),
          Nx.multiply(y1, x2)
        ),
        Nx.multiply(z1, w2)
      )

    tensor = Nx.stack([w, x, y, z])
    %__MODULE__{tensor: normalise_tensor(tensor)}
  end

  @doc """
  Returns the conjugate of a quaternion.

  For unit quaternions, the conjugate equals the inverse.

  ## Examples

      iex> q = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> qc = BB.Quaternion.conjugate(q)
      iex> Float.round(BB.Quaternion.z(qc), 6)
      -0.707107
  """
  @spec conjugate(t()) :: t()
  def conjugate(%__MODULE__{tensor: t}) do
    # Conjugate: negate the vector part
    w = t[0]
    xyz = Nx.negate(Nx.slice(t, [1], [3]))
    tensor = Nx.concatenate([Nx.reshape(w, {1}), xyz])

    %__MODULE__{tensor: tensor}
  end

  @doc """
  Normalises a quaternion to unit length.

  ## Examples

      iex> q = %BB.Quaternion{tensor: Nx.tensor([2.0, 0.0, 0.0, 0.0])}
      iex> qn = BB.Quaternion.normalise(q)
      iex> BB.Quaternion.w(qn)
      1.0
  """
  @spec normalise(t()) :: t()
  def normalise(%__MODULE__{tensor: t}) do
    %__MODULE__{tensor: normalise_tensor(t)}
  end

  defp normalise_tensor(tensor) do
    norm = Nx.LinAlg.norm(tensor)
    identity = Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64)

    Nx.select(
      Nx.greater(norm, 1.0e-10),
      Nx.divide(tensor, norm),
      identity
    )
  end

  @doc """
  Returns the inverse of a quaternion.

  For unit quaternions, this equals the conjugate.

  ## Examples

      iex> q = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> qi = BB.Quaternion.inverse(q)
      iex> qr = BB.Quaternion.multiply(q, qi)
      iex> Float.round(BB.Quaternion.w(qr), 6)
      1.0
  """
  @spec inverse(t()) :: t()
  def inverse(%__MODULE__{} = q) do
    conjugate(q)
  end

  @doc """
  Rotates a 3D vector by a quaternion.

  ## Examples

      iex> q = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> v = BB.Vec3.unit_x()
      iex> rotated = BB.Quaternion.rotate_vector(q, v)
      iex> {Float.round(BB.Vec3.x(rotated), 6), Float.round(BB.Vec3.y(rotated), 6)}
      {0.0, 1.0}
  """
  @spec rotate_vector(t(), Vec3.t()) :: Vec3.t()
  def rotate_vector(%__MODULE__{tensor: t}, %Vec3{tensor: v_tensor}) do
    w = t[0]
    u = Nx.slice(t, [1], [3])

    # Rodrigues rotation formula: v' = v + 2*w*(u x v) + 2*(u x (u x v))
    # Cross product: u x v
    u1 = u[0]
    u2 = u[1]
    u3 = u[2]
    v1 = v_tensor[0]
    v2 = v_tensor[1]
    v3 = v_tensor[2]

    # u x v
    uxv =
      Nx.stack([
        Nx.subtract(Nx.multiply(u2, v3), Nx.multiply(u3, v2)),
        Nx.subtract(Nx.multiply(u3, v1), Nx.multiply(u1, v3)),
        Nx.subtract(Nx.multiply(u1, v2), Nx.multiply(u2, v1))
      ])

    # u x (u x v)
    uxv1 = uxv[0]
    uxv2 = uxv[1]
    uxv3 = uxv[2]

    uuxv =
      Nx.stack([
        Nx.subtract(Nx.multiply(u2, uxv3), Nx.multiply(u3, uxv2)),
        Nx.subtract(Nx.multiply(u3, uxv1), Nx.multiply(u1, uxv3)),
        Nx.subtract(Nx.multiply(u1, uxv2), Nx.multiply(u2, uxv1))
      ])

    # v' = v + 2*w*(u x v) + 2*(u x (u x v))
    two = Nx.tensor(2.0, type: :f64)

    result =
      Nx.add(
        Nx.add(v_tensor, Nx.multiply(Nx.multiply(two, w), uxv)),
        Nx.multiply(two, uuxv)
      )

    Vec3.from_tensor(result)
  end

  @doc """
  Spherical linear interpolation between two quaternions.

  `t` should be between 0.0 and 1.0.

  ## Examples

      iex> q1 = BB.Quaternion.identity()
      iex> q2 = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi())
      iex> q_mid = BB.Quaternion.slerp(q1, q2, 0.5)
      iex> {_axis, angle} = BB.Quaternion.to_axis_angle(q_mid)
      iex> Float.round(angle, 6)
      1.570796
  """
  @spec slerp(t(), t(), number()) :: t()
  def slerp(%__MODULE__{tensor: t1}, %__MODULE__{tensor: t2}, t) when t >= 0 and t <= 1 do
    # Compute dot product
    dot = Nx.dot(t1, t2)

    # If dot is negative, negate one quaternion to take shorter path
    t2_adjusted = Nx.select(Nx.less(dot, 0), Nx.negate(t2), t2)
    dot_adjusted = Nx.abs(dot)

    # Clamp dot to valid range for acos
    dot_clamped = Nx.clip(dot_adjusted, 0.0, 1.0)

    # Check if quaternions are very close (use linear interpolation)
    close = Nx.greater(dot_clamped, 0.9995)

    # Linear interpolation path
    t_tensor = Nx.tensor(t, type: :f64)
    one_minus_t = Nx.subtract(1.0, t_tensor)
    lerp_result = Nx.add(Nx.multiply(t1, one_minus_t), Nx.multiply(t2_adjusted, t_tensor))

    # SLERP path
    theta = Nx.acos(dot_clamped)
    sin_theta = Nx.sin(theta)

    s1 = Nx.divide(Nx.sin(Nx.multiply(one_minus_t, theta)), sin_theta)
    s2 = Nx.divide(Nx.sin(Nx.multiply(t_tensor, theta)), sin_theta)

    slerp_result = Nx.add(Nx.multiply(t1, s1), Nx.multiply(t2_adjusted, s2))

    # Select based on closeness
    result = Nx.select(close, lerp_result, slerp_result)

    %__MODULE__{tensor: normalise_tensor(result)}
  end

  @doc """
  Computes the angular distance between two quaternions in radians.

  Returns a value between 0 and pi.

  ## Examples

      iex> q1 = BB.Quaternion.identity()
      iex> q2 = BB.Quaternion.from_axis_angle(BB.Vec3.unit_z(), :math.pi() / 2)
      iex> Float.round(BB.Quaternion.angular_distance(q1, q2), 6)
      1.570796
  """
  @spec angular_distance(t(), t()) :: float()
  def angular_distance(%__MODULE__{tensor: t1}, %__MODULE__{tensor: t2}) do
    # Compute absolute dot product (both q and -q represent same rotation)
    dot = Nx.abs(Nx.dot(t1, t2))

    # Clamp to valid range for acos
    dot_clamped = Nx.clip(dot, 0.0, 1.0)

    # Angular distance = 2 * acos(|dot|)
    angle = Nx.multiply(2.0, Nx.acos(dot_clamped))
    Nx.to_number(angle)
  end

  @doc """
  Converts to a list in XYZW order (for ROS/external system compatibility).

  ## Examples

      iex> q = BB.Quaternion.identity()
      iex> BB.Quaternion.to_xyzw_list(q)
      [0.0, 0.0, 0.0, 1.0]
  """
  @spec to_xyzw_list(t()) :: [float()]
  def to_xyzw_list(%__MODULE__{tensor: t}) do
    [Nx.to_number(t[1]), Nx.to_number(t[2]), Nx.to_number(t[3]), Nx.to_number(t[0])]
  end

  @doc """
  Creates from a list in XYZW order (for ROS/external system compatibility).

  ## Examples

      iex> q = BB.Quaternion.from_xyzw_list([0.0, 0.0, 0.0, 1.0])
      iex> BB.Quaternion.w(q)
      1.0
  """
  @spec from_xyzw_list([number()]) :: t()
  def from_xyzw_list([x, y, z, w]) do
    new(w, x, y, z)
  end

  @doc """
  Converts to a list in WXYZ order.

  ## Examples

      iex> q = BB.Quaternion.identity()
      iex> BB.Quaternion.to_list(q)
      [1.0, 0.0, 0.0, 0.0]
  """
  @spec to_list(t()) :: [float()]
  def to_list(%__MODULE__{tensor: t}) do
    Nx.to_flat_list(t)
  end

  @doc """
  Creates from a list in WXYZ order.

  ## Examples

      iex> q = BB.Quaternion.from_list([1.0, 0.0, 0.0, 0.0])
      iex> BB.Quaternion.w(q)
      1.0
  """
  @spec from_list([number()]) :: t()
  def from_list([w, x, y, z]) do
    new(w, x, y, z)
  end
end
