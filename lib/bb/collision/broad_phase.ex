# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision.BroadPhase do
  @moduledoc """
  Broad phase collision detection using Axis-Aligned Bounding Boxes (AABBs).

  The broad phase is a fast culling step that eliminates pairs of objects that
  cannot possibly be colliding. This reduces the number of expensive narrow-phase
  collision tests required.

  AABBs are simple boxes aligned to the world axes, making overlap tests very fast.
  They may be larger than the actual geometry (especially for rotated objects),
  so a positive broad phase result only indicates *potential* collision.
  """

  alias BB.Collision.Mesh
  alias BB.Math.{Quaternion, Transform, Vec3}

  @type aabb :: {min :: Vec3.t(), max :: Vec3.t()}

  @doc """
  Check if two AABBs overlap.

  This is a very fast O(1) test - two AABBs overlap if and only if they overlap
  on all three axes.
  """
  @spec overlap?(aabb(), aabb()) :: boolean()
  def overlap?({min1, max1}, {min2, max2}) do
    Vec3.x(min1) <= Vec3.x(max2) and
      Vec3.x(max1) >= Vec3.x(min2) and
      Vec3.y(min1) <= Vec3.y(max2) and
      Vec3.y(max1) >= Vec3.y(min2) and
      Vec3.z(min1) <= Vec3.z(max2) and
      Vec3.z(max1) >= Vec3.z(min2)
  end

  @doc """
  Compute the AABB for a collision geometry in world space.

  Takes a geometry specification (as stored in Robot.Link) and a transform
  representing the geometry's position and orientation in world space.

  ## Supported Geometry Types

  - `{:sphere, %{radius: float()}}` - Sphere
  - `{:capsule, %{radius: float(), length: float()}}` - Capsule (cylinder with spherical caps)
  - `{:cylinder, %{radius: float(), height: float()}}` - Cylinder (treated as capsule)
  - `{:box, %{x: float(), y: float(), z: float()}}` - Box

  ## Examples

      iex> geometry = {:sphere, %{radius: 1.0}}
      iex> transform = Transform.identity()
      iex> {min, max} = BB.Collision.BroadPhase.compute_aabb(geometry, transform)
      iex> {Vec3.x(min), Vec3.x(max)}
      {-1.0, 1.0}
  """
  @spec compute_aabb(BB.Robot.Link.geometry(), Transform.t()) :: aabb()
  def compute_aabb({:sphere, %{radius: r}}, transform) do
    centre = Transform.get_translation(transform)
    offset = Vec3.new(r, r, r)
    {Vec3.subtract(centre, offset), Vec3.add(centre, offset)}
  end

  def compute_aabb({:capsule, %{radius: r, length: l}}, transform) do
    # Capsule extends along local Z axis
    # Total length is l (cylinder) + 2*r (hemispherical caps)
    half_length = l / 2

    centre = Transform.get_translation(transform)
    orientation = Transform.get_quaternion(transform)

    # Local endpoints (along Z axis)
    local_a = Vec3.new(0.0, 0.0, -half_length)
    local_b = Vec3.new(0.0, 0.0, half_length)

    # Transform to world space
    world_a = Vec3.add(centre, Quaternion.rotate_vector(orientation, local_a))
    world_b = Vec3.add(centre, Quaternion.rotate_vector(orientation, local_b))

    # AABB is the bounds of both endpoints expanded by radius
    min_pt =
      Vec3.new(
        min(Vec3.x(world_a), Vec3.x(world_b)) - r,
        min(Vec3.y(world_a), Vec3.y(world_b)) - r,
        min(Vec3.z(world_a), Vec3.z(world_b)) - r
      )

    max_pt =
      Vec3.new(
        max(Vec3.x(world_a), Vec3.x(world_b)) + r,
        max(Vec3.y(world_a), Vec3.y(world_b)) + r,
        max(Vec3.z(world_a), Vec3.z(world_b)) + r
      )

    {min_pt, max_pt}
  end

  def compute_aabb({:cylinder, %{radius: r, height: h}}, transform) do
    # Treat cylinder as capsule for conservative bounding
    compute_aabb({:capsule, %{radius: r, length: h}}, transform)
  end

  def compute_aabb({:box, %{x: hx, y: hy, z: hz}}, transform) do
    # For an oriented box, we need to transform all 8 corners and find bounds
    centre = Transform.get_translation(transform)
    orientation = Transform.get_quaternion(transform)

    # Half extents in each direction
    corners = [
      Vec3.new(hx, hy, hz),
      Vec3.new(hx, hy, -hz),
      Vec3.new(hx, -hy, hz),
      Vec3.new(hx, -hy, -hz),
      Vec3.new(-hx, hy, hz),
      Vec3.new(-hx, hy, -hz),
      Vec3.new(-hx, -hy, hz),
      Vec3.new(-hx, -hy, -hz)
    ]

    # Transform corners to world space
    world_corners =
      Enum.map(corners, fn local ->
        Vec3.add(centre, Quaternion.rotate_vector(orientation, local))
      end)

    # Find min/max
    xs = Enum.map(world_corners, &Vec3.x/1)
    ys = Enum.map(world_corners, &Vec3.y/1)
    zs = Enum.map(world_corners, &Vec3.z/1)

    {Vec3.new(Enum.min(xs), Enum.min(ys), Enum.min(zs)),
     Vec3.new(Enum.max(xs), Enum.max(ys), Enum.max(zs))}
  end

  def compute_aabb({:mesh, %{filename: filename, scale: scale}}, transform) do
    case Mesh.load_bounds(filename) do
      {:ok, bounds} ->
        # Transform the mesh AABB to world space
        transform_mesh_aabb(bounds.aabb, scale, transform)

      {:error, _} ->
        # Fall back to placeholder (unit sphere AABB)
        centre = Transform.get_translation(transform)
        offset = Vec3.new(1.0, 1.0, 1.0)
        {Vec3.subtract(centre, offset), Vec3.add(centre, offset)}
    end
  end

  def compute_aabb({:mesh, _mesh_data}, transform) do
    # Mesh without filename - use placeholder
    centre = Transform.get_translation(transform)
    offset = Vec3.new(1.0, 1.0, 1.0)
    {Vec3.subtract(centre, offset), Vec3.add(centre, offset)}
  end

  defp transform_mesh_aabb({local_min, local_max}, scale, transform) do
    # Scale the local AABB
    scaled_min = Vec3.scale(local_min, scale)
    scaled_max = Vec3.scale(local_max, scale)

    centre = Transform.get_translation(transform)
    orientation = Transform.get_quaternion(transform)

    # Get all 8 corners of the scaled local AABB
    corners = [
      Vec3.new(Vec3.x(scaled_min), Vec3.y(scaled_min), Vec3.z(scaled_min)),
      Vec3.new(Vec3.x(scaled_min), Vec3.y(scaled_min), Vec3.z(scaled_max)),
      Vec3.new(Vec3.x(scaled_min), Vec3.y(scaled_max), Vec3.z(scaled_min)),
      Vec3.new(Vec3.x(scaled_min), Vec3.y(scaled_max), Vec3.z(scaled_max)),
      Vec3.new(Vec3.x(scaled_max), Vec3.y(scaled_min), Vec3.z(scaled_min)),
      Vec3.new(Vec3.x(scaled_max), Vec3.y(scaled_min), Vec3.z(scaled_max)),
      Vec3.new(Vec3.x(scaled_max), Vec3.y(scaled_max), Vec3.z(scaled_min)),
      Vec3.new(Vec3.x(scaled_max), Vec3.y(scaled_max), Vec3.z(scaled_max))
    ]

    # Transform corners to world space
    world_corners =
      Enum.map(corners, fn local ->
        Vec3.add(centre, Quaternion.rotate_vector(orientation, local))
      end)

    # Find world-space AABB
    xs = Enum.map(world_corners, &Vec3.x/1)
    ys = Enum.map(world_corners, &Vec3.y/1)
    zs = Enum.map(world_corners, &Vec3.z/1)

    {Vec3.new(Enum.min(xs), Enum.min(ys), Enum.min(zs)),
     Vec3.new(Enum.max(xs), Enum.max(ys), Enum.max(zs))}
  end

  @doc """
  Expand an AABB by a margin in all directions.

  Useful for adding safety buffers to collision checks.
  """
  @spec expand(aabb(), float()) :: aabb()
  def expand({min_pt, max_pt}, margin) do
    offset = Vec3.new(margin, margin, margin)
    {Vec3.subtract(min_pt, offset), Vec3.add(max_pt, offset)}
  end

  @doc """
  Merge two AABBs into a single AABB that contains both.
  """
  @spec merge(aabb(), aabb()) :: aabb()
  def merge({min1, max1}, {min2, max2}) do
    {Vec3.new(
       min(Vec3.x(min1), Vec3.x(min2)),
       min(Vec3.y(min1), Vec3.y(min2)),
       min(Vec3.z(min1), Vec3.z(min2))
     ),
     Vec3.new(
       max(Vec3.x(max1), Vec3.x(max2)),
       max(Vec3.y(max1), Vec3.y(max2)),
       max(Vec3.z(max1), Vec3.z(max2))
     )}
  end

  @doc """
  Compute the centre point of an AABB.
  """
  @spec centre(aabb()) :: Vec3.t()
  def centre({min_pt, max_pt}) do
    Vec3.lerp(min_pt, max_pt, 0.5)
  end

  @doc """
  Compute the size (extents) of an AABB in each dimension.
  """
  @spec size(aabb()) :: Vec3.t()
  def size({min_pt, max_pt}) do
    Vec3.subtract(max_pt, min_pt)
  end

  @doc """
  Check if a point is inside an AABB.
  """
  @spec contains_point?(aabb(), Vec3.t()) :: boolean()
  def contains_point?({min_pt, max_pt}, point) do
    Vec3.x(point) >= Vec3.x(min_pt) and Vec3.x(point) <= Vec3.x(max_pt) and
      Vec3.y(point) >= Vec3.y(min_pt) and Vec3.y(point) <= Vec3.y(max_pt) and
      Vec3.z(point) >= Vec3.z(min_pt) and Vec3.z(point) <= Vec3.z(max_pt)
  end
end
