# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Transform2D do
  @moduledoc """
  A rigid transform within a plane: translation and rotation about the plane's normal.

  This is the configuration of a `:planar` joint — two translations in the plane
  and one rotation about its normal, which is the joint's `axis`.

  ## Not tensor-backed

  Unlike `BB.Math.Transform`, this holds three plain floats rather than an Nx
  tensor. `BB.Math.Transform` needs a matrix because it composes through
  kinematic chains by matrix multiply; a planar configuration does not, because
  it is lifted into a `BB.Math.Transform` by `to_transform/2` before it reaches
  forward kinematics.

  Storing `theta` as an angle rather than as the `sin`/`cos` entries of a
  rotation matrix means there is no orthonormality to lose and no
  renormalisation on read.

  ## Conventions

  - `x` and `y` are in metres, within the plane
  - `theta` is in radians, right-handed about the plane's normal

  ## Examples

      iex> t = BB.Math.Transform2D.new(1.0, 2.0, :math.pi() / 2)
      iex> {t.x, t.y}
      {1.0, 2.0}
  """

  alias BB.Math.Transform
  alias BB.Math.Vec3

  defstruct [:x, :y, :theta]

  @type t :: %__MODULE__{x: float(), y: float(), theta: float()}

  @doc """
  Create a planar transform from in-plane translation and rotation.

  ## Examples

      iex> BB.Math.Transform2D.new(1, 2, 0)
      %BB.Math.Transform2D{x: 1.0, y: 2.0, theta: 0.0}
  """
  @spec new(number(), number(), number()) :: t()
  def new(x, y, theta) when is_number(x) and is_number(y) and is_number(theta) do
    %__MODULE__{x: x / 1, y: y / 1, theta: theta / 1}
  end

  @doc """
  The identity planar transform.

  ## Examples

      iex> BB.Math.Transform2D.identity()
      %BB.Math.Transform2D{x: 0.0, y: 0.0, theta: 0.0}
  """
  @spec identity() :: t()
  def identity, do: %__MODULE__{x: 0.0, y: 0.0, theta: 0.0}

  @doc """
  Compose two planar transforms.

  `compose(a, b)` returns the transform that applies `a` first, then `b`,
  matching `BB.Math.Transform.compose/2`.

  ## Examples

      iex> a = BB.Math.Transform2D.new(1.0, 0.0, :math.pi() / 2)
      iex> b = BB.Math.Transform2D.new(1.0, 0.0, 0.0)
      iex> c = BB.Math.Transform2D.compose(a, b)
      iex> {Float.round(c.x, 6), Float.round(c.y, 6)}
      {1.0, 1.0}
  """
  @spec compose(t(), t()) :: t()
  def compose(%__MODULE__{} = a, %__MODULE__{} = b) do
    c = :math.cos(a.theta)
    s = :math.sin(a.theta)

    %__MODULE__{
      x: a.x + c * b.x - s * b.y,
      y: a.y + s * b.x + c * b.y,
      theta: a.theta + b.theta
    }
  end

  @doc """
  Invert a planar transform.

  ## Examples

      iex> t = BB.Math.Transform2D.new(1.0, 2.0, 0.5)
      iex> c = BB.Math.Transform2D.compose(t, BB.Math.Transform2D.inverse(t))
      iex> {abs(c.x) < 1.0e-12, abs(c.y) < 1.0e-12, abs(c.theta) < 1.0e-12}
      {true, true, true}
  """
  @spec inverse(t()) :: t()
  def inverse(%__MODULE__{} = t) do
    c = :math.cos(t.theta)
    s = :math.sin(t.theta)

    %__MODULE__{
      x: -(c * t.x) - s * t.y,
      y: s * t.x - c * t.y,
      theta: -t.theta
    }
  end

  @doc """
  Lift into 3D, given the normal of the plane the transform is in.

  The normal is the `:planar` joint's `axis`. It is taken as a parameter rather
  than carried on the struct so that the joint remains the single source of
  truth for its own plane — a configuration carrying a copy could disagree with
  the joint that owns it.

  The plane is spanned by two axes perpendicular to `normal`, with `x` along the
  first and `y` along the second, and `theta` is a right-handed rotation about
  `normal`. For the canonical `{0, 0, 1}` normal this reduces to the XY plane
  with `theta` as yaw.

  ## Examples

      iex> t2d = BB.Math.Transform2D.new(1.0, 2.0, 0.0)
      iex> t = BB.Math.Transform2D.to_transform(t2d, BB.Math.Vec3.unit_z())
      iex> BB.Math.Transform.get_translation(t) |> BB.Math.Vec3.to_list()
      [1.0, 2.0, 0.0]
  """
  @spec to_transform(t(), Vec3.t()) :: Transform.t()
  def to_transform(%__MODULE__{} = t, %Vec3{} = normal) do
    n = Vec3.normalise(normal)
    {u, v} = basis(n)

    offset = Vec3.add(Vec3.scale(u, t.x), Vec3.scale(v, t.y))

    offset
    |> Transform.translation()
    |> Transform.compose(Transform.from_axis_angle(n, t.theta))
  end

  @doc """
  The two in-plane axes for a given plane normal.

  `{u, v, normal}` is right-handed with `u × v == normal`, `u` is the direction
  `x` measures along and `v` the direction `y` measures along. The canonical
  `{0, 0, 1}` normal reduces to the XY plane with `u == x̂` and `v == ŷ`.

  Exposed because a planar joint's Jacobian columns must be expressed in the same
  basis `to_transform/2` lifts its configuration through, and the two disagreeing
  would be a silently wrong derivative.

  ## Examples

      iex> {u, v} = BB.Math.Transform2D.plane_basis(BB.Math.Vec3.unit_z())
      iex> {BB.Math.Vec3.to_list(u), BB.Math.Vec3.to_list(v)}
      {[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]}
  """
  @spec plane_basis(Vec3.t()) :: {Vec3.t(), Vec3.t()}
  def plane_basis(%Vec3{} = normal) do
    basis(Vec3.normalise(normal))
  end

  # The seed is whichever cardinal axis is least aligned with the normal, which
  # keeps the cross product well-conditioned.
  defp basis(%Vec3{} = n) do
    seed =
      if abs(Vec3.z(n)) < 0.9 do
        Vec3.unit_z()
      else
        Vec3.unit_y()
      end

    u = seed |> Vec3.cross(n) |> Vec3.normalise()
    v = n |> Vec3.cross(u) |> Vec3.normalise()

    {u, v}
  end
end
