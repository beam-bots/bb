# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision do
  @moduledoc """
  Collision detection for BB robots.

  This module provides self-collision detection and environment collision detection,
  following the same architectural pattern as `BB.Robot.Kinematics`.

  ## Self-Collision Detection

  Self-collision checking determines if any parts of the robot are colliding with
  each other. Adjacent links (connected by a joint) are automatically excluded from
  collision checks since they are expected to be in contact.

      # Quick boolean check
      BB.Collision.self_collision?(robot, positions)

      # Detailed collision information
      BB.Collision.detect_self_collisions(robot, positions)

  ## Environment Collision Detection

  Environment collision checks if the robot collides with obstacles in the workspace.

      obstacles = [
        BB.Collision.obstacle(:box, centre, half_extents),
        BB.Collision.obstacle(:sphere, centre, radius)
      ]

      BB.Collision.collides_with?(robot, positions, obstacles)

  ## Performance

  Collision detection uses a two-phase approach:
  1. **Broad phase**: Fast AABB overlap tests to eliminate non-colliding pairs
  2. **Narrow phase**: Precise primitive intersection tests for potential collisions

  For a typical 6-DOF robot, self-collision checks complete in under 1ms.
  """

  alias BB.Robot
  alias BB.Robot.Kinematics
  alias BB.Collision.{BroadPhase, Primitives}
  alias BB.Math.{Transform, Vec3, Quaternion}

  @type positions :: %{atom() => float()}

  @type collision_info :: %{
          link_a: atom(),
          link_b: atom() | :environment,
          collision_a: atom() | nil,
          collision_b: atom() | nil,
          penetration_depth: float()
        }

  @type obstacle :: %{
          type: :sphere | :capsule | :box,
          geometry: Primitives.geometry(),
          aabb: BroadPhase.aabb()
        }

  # ============================================================================
  # Self-Collision Detection
  # ============================================================================

  @doc """
  Check if the robot is in self-collision at the given joint positions.

  Returns `true` if any non-adjacent links are colliding, `false` otherwise.

  ## Options

  - `:margin` - Safety margin in metres added to all geometries (default: 0.0)

  ## Examples

      positions = %{shoulder: 0.0, elbow: 1.57, wrist: 0.0}
      BB.Collision.self_collision?(robot, positions)
      # => false

      BB.Collision.self_collision?(robot, positions, margin: 0.01)
      # => true  (with 1cm safety margin)
  """
  @spec self_collision?(Robot.t(), positions(), keyword()) :: boolean()
  def self_collision?(%Robot{} = robot, positions, opts \\ []) when is_map(positions) do
    case detect_self_collisions(robot, positions, opts) do
      [] -> false
      [_ | _] -> true
    end
  end

  @doc """
  Detect all self-collisions at the given joint positions.

  Returns a list of collision info maps describing each collision. Adjacent links
  (connected by a joint) are excluded from checks.

  ## Options

  - `:margin` - Safety margin in metres added to all geometries (default: 0.0)

  ## Examples

      collisions = BB.Collision.detect_self_collisions(robot, positions)
      # => [%{link_a: :forearm, link_b: :base, collision_a: nil, collision_b: nil, penetration_depth: 0.02}]
  """
  @spec detect_self_collisions(Robot.t(), positions(), keyword()) :: [collision_info()]
  def detect_self_collisions(%Robot{} = robot, positions, opts \\ []) when is_map(positions) do
    margin = Keyword.get(opts, :margin, 0.0)

    # Get all link transforms in world space
    transforms = Kinematics.all_link_transforms(robot, positions)

    # Build adjacency set (pairs of links that should not be checked)
    adjacent = build_adjacency_set(robot)

    # Get all links with collision geometry
    links_with_collisions =
      robot.links
      |> Map.values()
      |> Enum.filter(fn link -> link.collisions != [] end)

    # Compute AABBs for all collision geometries
    link_aabbs = compute_link_aabbs(links_with_collisions, transforms, margin)

    # Generate all non-adjacent pairs
    pairs = generate_collision_pairs(links_with_collisions, adjacent)

    # Two-phase collision detection
    pairs
    |> Enum.flat_map(fn {link_a, link_b} ->
      check_link_pair(link_a, link_b, transforms, link_aabbs, margin)
    end)
  end

  # ============================================================================
  # Environment Collision Detection
  # ============================================================================

  @doc """
  Check if the robot collides with any obstacles at the given joint positions.

  ## Options

  - `:margin` - Safety margin in metres (default: 0.0)

  ## Examples

      obstacles = [BB.Collision.obstacle(:sphere, Vec3.new(0.5, 0, 0.3), 0.1)]
      BB.Collision.collides_with?(robot, positions, obstacles)
  """
  @spec collides_with?(Robot.t(), positions(), [obstacle()], keyword()) :: boolean()
  def collides_with?(%Robot{} = robot, positions, obstacles, opts \\ []) do
    case detect_collisions(robot, positions, obstacles, opts) do
      [] -> false
      [_ | _] -> true
    end
  end

  @doc """
  Detect all collisions between the robot and environment obstacles.

  ## Options

  - `:margin` - Safety margin in metres (default: 0.0)
  """
  @spec detect_collisions(Robot.t(), positions(), [obstacle()], keyword()) :: [collision_info()]
  def detect_collisions(%Robot{} = robot, positions, obstacles, opts \\ []) do
    margin = Keyword.get(opts, :margin, 0.0)

    transforms = Kinematics.all_link_transforms(robot, positions)

    links_with_collisions =
      robot.links
      |> Map.values()
      |> Enum.filter(fn link -> link.collisions != [] end)

    link_aabbs = compute_link_aabbs(links_with_collisions, transforms, margin)

    links_with_collisions
    |> Enum.flat_map(fn link ->
      check_link_obstacles(link, obstacles, transforms, link_aabbs, margin)
    end)
  end

  # ============================================================================
  # Obstacle Creation
  # ============================================================================

  @doc """
  Create an obstacle for environment collision detection.

  ## Sphere

      obstacle = BB.Collision.obstacle(:sphere, centre, radius)

  ## Capsule

      obstacle = BB.Collision.obstacle(:capsule, point_a, point_b, radius)

  ## Axis-Aligned Box

      obstacle = BB.Collision.obstacle(:box, centre, half_extents)

  ## Oriented Box

      rotation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 4)
      obstacle = BB.Collision.obstacle(:box, centre, half_extents, rotation)
  """
  @spec obstacle(:sphere, Vec3.t(), float()) :: obstacle()
  @spec obstacle(:capsule, Vec3.t(), Vec3.t(), float()) :: obstacle()
  @spec obstacle(:box, Vec3.t(), Vec3.t()) :: obstacle()
  @spec obstacle(:box, Vec3.t(), Vec3.t(), Quaternion.t()) :: obstacle()
  def obstacle(:sphere, centre, radius) do
    geometry = {:sphere, centre, radius}
    aabb = sphere_aabb(centre, radius)
    %{type: :sphere, geometry: geometry, aabb: aabb}
  end

  def obstacle(:box, centre, half_extents) do
    axes = {Vec3.unit_x(), Vec3.unit_y(), Vec3.unit_z()}
    geometry = {:box, centre, half_extents, axes}
    aabb = box_aabb(centre, half_extents)
    %{type: :box, geometry: geometry, aabb: aabb}
  end

  def obstacle(:capsule, point_a, point_b, radius) do
    geometry = {:capsule, point_a, point_b, radius}
    aabb = capsule_aabb(point_a, point_b, radius)
    %{type: :capsule, geometry: geometry, aabb: aabb}
  end

  def obstacle(:box, centre, half_extents, orientation) do
    ax = Quaternion.rotate_vector(orientation, Vec3.unit_x())
    ay = Quaternion.rotate_vector(orientation, Vec3.unit_y())
    az = Quaternion.rotate_vector(orientation, Vec3.unit_z())

    geometry = {:box, centre, half_extents, {ax, ay, az}}
    aabb = oriented_box_aabb(centre, half_extents, {ax, ay, az})
    %{type: :box, geometry: geometry, aabb: aabb}
  end

  # ============================================================================
  # Adjacency
  # ============================================================================

  @doc """
  Build a set of adjacent link pairs from the robot topology.

  Adjacent links are connected by a joint and should not be checked for collision.
  """
  @spec build_adjacency_set(Robot.t()) :: MapSet.t({atom(), atom()})
  def build_adjacency_set(%Robot{} = robot) do
    robot.joints
    |> Map.values()
    |> Enum.reduce(MapSet.new(), fn joint, set ->
      # Add both orderings for easy lookup
      set
      |> MapSet.put({joint.parent_link, joint.child_link})
      |> MapSet.put({joint.child_link, joint.parent_link})
    end)
  end

  # ============================================================================
  # Private - Link Pair Checking
  # ============================================================================

  defp generate_collision_pairs(links, adjacent) do
    for link_a <- links,
        link_b <- links,
        link_a.name < link_b.name,
        not MapSet.member?(adjacent, {link_a.name, link_b.name}) do
      {link_a, link_b}
    end
  end

  defp check_link_pair(link_a, link_b, transforms, link_aabbs, margin) do
    aabbs_a = Map.get(link_aabbs, link_a.name, [])
    aabbs_b = Map.get(link_aabbs, link_b.name, [])

    transform_a = Map.fetch!(transforms, link_a.name)
    transform_b = Map.fetch!(transforms, link_b.name)

    for {coll_a, aabb_a} <- aabbs_a,
        {coll_b, aabb_b} <- aabbs_b,
        BroadPhase.overlap?(aabb_a, aabb_b),
        collision = narrow_phase_check(coll_a, coll_b, transform_a, transform_b, margin),
        collision != nil do
      %{
        link_a: link_a.name,
        link_b: link_b.name,
        collision_a: coll_a.name,
        collision_b: coll_b.name,
        penetration_depth: collision.penetration_depth
      }
    end
  end

  defp check_link_obstacles(link, obstacles, transforms, link_aabbs, margin) do
    link_collision_aabbs = Map.get(link_aabbs, link.name, [])
    transform = Map.fetch!(transforms, link.name)

    for {coll, link_aabb} <- link_collision_aabbs,
        obstacle <- obstacles,
        expanded_obstacle_aabb = BroadPhase.expand(obstacle.aabb, margin),
        BroadPhase.overlap?(link_aabb, expanded_obstacle_aabb),
        collision = narrow_phase_obstacle(coll, obstacle, transform, margin),
        collision != nil do
      %{
        link_a: link.name,
        link_b: :environment,
        collision_a: coll.name,
        collision_b: nil,
        penetration_depth: collision.penetration_depth
      }
    end
  end

  # ============================================================================
  # Private - AABB Computation
  # ============================================================================

  defp compute_link_aabbs(links, transforms, margin) do
    links
    |> Enum.map(fn link ->
      transform = Map.fetch!(transforms, link.name)

      aabbs =
        link.collisions
        |> Enum.filter(fn coll -> coll.geometry != nil end)
        |> Enum.map(fn coll ->
          coll_transform = compose_collision_transform(transform, coll.origin)
          aabb = BroadPhase.compute_aabb(coll.geometry, coll_transform)
          expanded_aabb = BroadPhase.expand(aabb, margin)
          {coll, expanded_aabb}
        end)

      {link.name, aabbs}
    end)
    |> Map.new()
  end

  defp compose_collision_transform(link_transform, nil), do: link_transform

  defp compose_collision_transform(link_transform, {pos, orient}) do
    local_transform = Transform.from_origin(%{position: pos, orientation: orient})
    Transform.compose(link_transform, local_transform)
  end

  # ============================================================================
  # Private - Narrow Phase
  # ============================================================================

  defp narrow_phase_check(coll_a, coll_b, transform_a, transform_b, margin) do
    geom_a = to_world_geometry(coll_a, transform_a)
    geom_b = to_world_geometry(coll_b, transform_b)

    case Primitives.test_with_margin(geom_a, geom_b, margin) do
      {:collision, depth} -> %{penetration_depth: depth}
      :no_collision -> nil
    end
  end

  defp narrow_phase_obstacle(coll, obstacle, transform, margin) do
    geom = to_world_geometry(coll, transform)

    case Primitives.test_with_margin(geom, obstacle.geometry, margin) do
      {:collision, depth} -> %{penetration_depth: depth}
      :no_collision -> nil
    end
  end

  defp to_world_geometry(collision, link_transform) do
    coll_transform = compose_collision_transform(link_transform, collision.origin)
    geometry_to_primitive(collision.geometry, coll_transform)
  end

  defp geometry_to_primitive({:sphere, %{radius: r}}, transform) do
    centre = Transform.get_translation(transform)
    {:sphere, centre, r}
  end

  defp geometry_to_primitive({:capsule, %{radius: r, length: l}}, transform) do
    centre = Transform.get_translation(transform)
    orientation = Transform.get_quaternion(transform)

    half_length = l / 2
    local_a = Vec3.new(0.0, 0.0, -half_length)
    local_b = Vec3.new(0.0, 0.0, half_length)

    point_a = Vec3.add(centre, Quaternion.rotate_vector(orientation, local_a))
    point_b = Vec3.add(centre, Quaternion.rotate_vector(orientation, local_b))

    {:capsule, point_a, point_b, r}
  end

  defp geometry_to_primitive({:cylinder, %{radius: r, height: h}}, transform) do
    # Treat cylinder as capsule
    geometry_to_primitive({:capsule, %{radius: r, length: h}}, transform)
  end

  defp geometry_to_primitive({:box, %{x: hx, y: hy, z: hz}}, transform) do
    centre = Transform.get_translation(transform)
    orientation = Transform.get_quaternion(transform)

    ax = Quaternion.rotate_vector(orientation, Vec3.unit_x())
    ay = Quaternion.rotate_vector(orientation, Vec3.unit_y())
    az = Quaternion.rotate_vector(orientation, Vec3.unit_z())

    {:box, centre, Vec3.new(hx, hy, hz), {ax, ay, az}}
  end

  defp geometry_to_primitive({:mesh, %{filename: filename, scale: scale}}, transform) do
    # Use bounding sphere for mesh collision
    case BB.Collision.Mesh.load_bounds(filename) do
      {:ok, bounds} ->
        {local_centre, radius} = bounds.bounding_sphere
        centre = Transform.get_translation(transform)
        orientation = Transform.get_quaternion(transform)

        scaled_local = Vec3.scale(local_centre, scale)
        world_centre = Vec3.add(centre, Quaternion.rotate_vector(orientation, scaled_local))

        {:sphere, world_centre, radius * scale}

      {:error, _} ->
        # Fallback: unit sphere at transform origin
        centre = Transform.get_translation(transform)
        {:sphere, centre, 1.0}
    end
  end

  defp geometry_to_primitive({:mesh, _}, transform) do
    centre = Transform.get_translation(transform)
    {:sphere, centre, 1.0}
  end

  # ============================================================================
  # Private - Obstacle AABBs
  # ============================================================================

  defp sphere_aabb(centre, radius) do
    offset = Vec3.new(radius, radius, radius)
    {Vec3.subtract(centre, offset), Vec3.add(centre, offset)}
  end

  defp capsule_aabb(point_a, point_b, radius) do
    min_pt =
      Vec3.new(
        min(Vec3.x(point_a), Vec3.x(point_b)) - radius,
        min(Vec3.y(point_a), Vec3.y(point_b)) - radius,
        min(Vec3.z(point_a), Vec3.z(point_b)) - radius
      )

    max_pt =
      Vec3.new(
        max(Vec3.x(point_a), Vec3.x(point_b)) + radius,
        max(Vec3.y(point_a), Vec3.y(point_b)) + radius,
        max(Vec3.z(point_a), Vec3.z(point_b)) + radius
      )

    {min_pt, max_pt}
  end

  defp box_aabb(centre, half_extents) do
    {Vec3.subtract(centre, half_extents), Vec3.add(centre, half_extents)}
  end

  defp oriented_box_aabb(centre, half_extents, {ax, ay, az}) do
    # Project half-extents onto world axes
    hx = Vec3.x(half_extents)
    hy = Vec3.y(half_extents)
    hz = Vec3.z(half_extents)

    extent_x = hx * abs(Vec3.x(ax)) + hy * abs(Vec3.x(ay)) + hz * abs(Vec3.x(az))
    extent_y = hx * abs(Vec3.y(ax)) + hy * abs(Vec3.y(ay)) + hz * abs(Vec3.y(az))
    extent_z = hx * abs(Vec3.z(ax)) + hy * abs(Vec3.z(ay)) + hz * abs(Vec3.z(az))

    world_extent = Vec3.new(extent_x, extent_y, extent_z)
    {Vec3.subtract(centre, world_extent), Vec3.add(centre, world_extent)}
  end
end
