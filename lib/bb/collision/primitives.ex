# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision.Primitives do
  @moduledoc """
  Collision detection algorithms for primitive geometry pairs.

  All functions take world-space geometry (position + orientation applied)
  and return either `{:collision, penetration_depth}` or `:no_collision`.

  Penetration depth is the estimated overlap distance - how far the geometries
  would need to be separated to no longer collide.

  ## Supported Geometry Types

  - Sphere: `{:sphere, centre :: Vec3.t(), radius :: float()}`
  - Capsule: `{:capsule, point_a :: Vec3.t(), point_b :: Vec3.t(), radius :: float()}`
  - Box (OBB): `{:box, centre :: Vec3.t(), half_extents :: Vec3.t(), axes :: {Vec3.t(), Vec3.t(), Vec3.t()}}`

  Cylinders are converted to capsules internally for simpler, more conservative collision detection.
  """

  alias BB.Math.Vec3

  @type sphere :: {:sphere, centre :: Vec3.t(), radius :: float()}
  @type capsule :: {:capsule, point_a :: Vec3.t(), point_b :: Vec3.t(), radius :: float()}
  @type box :: {:box, centre :: Vec3.t(), half_extents :: Vec3.t(), axes :: {Vec3.t(), Vec3.t(), Vec3.t()}}
  @type geometry :: sphere() | capsule() | box()

  @type collision_result :: {:collision, penetration_depth :: float()} | :no_collision

  # ============================================================================
  # Public API
  # ============================================================================

  @doc """
  Test two geometries for collision.

  Dispatches to the appropriate collision test based on geometry types.
  Order of arguments doesn't matter - the function handles symmetry internally.

  ## Examples

      iex> sphere1 = {:sphere, Vec3.new(0, 0, 0), 1.0}
      iex> sphere2 = {:sphere, Vec3.new(1.5, 0, 0), 1.0}
      iex> BB.Collision.Primitives.test(sphere1, sphere2)
      {:collision, 0.5}

      iex> sphere1 = {:sphere, Vec3.new(0, 0, 0), 1.0}
      iex> sphere2 = {:sphere, Vec3.new(3.0, 0, 0), 1.0}
      iex> BB.Collision.Primitives.test(sphere1, sphere2)
      :no_collision
  """
  @spec test(geometry(), geometry()) :: collision_result()
  def test({:sphere, _, _} = a, {:sphere, _, _} = b), do: sphere_sphere(a, b)
  def test({:capsule, _, _, _} = a, {:capsule, _, _, _} = b), do: capsule_capsule(a, b)
  def test({:box, _, _, _} = a, {:box, _, _, _} = b), do: box_box(a, b)

  def test({:sphere, _, _} = a, {:capsule, _, _, _} = b), do: sphere_capsule(a, b)
  def test({:capsule, _, _, _} = a, {:sphere, _, _} = b), do: sphere_capsule(b, a)

  def test({:sphere, _, _} = a, {:box, _, _, _} = b), do: sphere_box(a, b)
  def test({:box, _, _, _} = a, {:sphere, _, _} = b), do: sphere_box(b, a)

  def test({:capsule, _, _, _} = a, {:box, _, _, _} = b), do: capsule_box(a, b)
  def test({:box, _, _, _} = a, {:capsule, _, _, _} = b), do: capsule_box(b, a)

  @doc """
  Test two geometries with an additional margin/padding.

  The margin is added to both geometries, effectively expanding them.
  Useful for detecting "near misses" or adding safety buffers.
  """
  @spec test_with_margin(geometry(), geometry(), margin :: float()) :: collision_result()
  def test_with_margin(a, b, margin) when margin > 0 do
    a_expanded = expand_geometry(a, margin)
    b_expanded = expand_geometry(b, margin)
    test(a_expanded, b_expanded)
  end

  def test_with_margin(a, b, _margin), do: test(a, b)

  # ============================================================================
  # Sphere-Sphere Collision
  # ============================================================================

  @doc """
  Test collision between two spheres.

  Two spheres collide if the distance between their centres is less than
  the sum of their radii.
  """
  @spec sphere_sphere(sphere(), sphere()) :: collision_result()
  def sphere_sphere({:sphere, c1, r1}, {:sphere, c2, r2}) do
    distance = Vec3.distance(c1, c2)
    sum_radii = r1 + r2

    if distance < sum_radii do
      {:collision, sum_radii - distance}
    else
      :no_collision
    end
  end

  # ============================================================================
  # Capsule-Capsule Collision
  # ============================================================================

  @doc """
  Test collision between two capsules.

  Two capsules collide if the closest distance between their line segments
  is less than the sum of their radii.
  """
  @spec capsule_capsule(capsule(), capsule()) :: collision_result()
  def capsule_capsule({:capsule, a1, b1, r1}, {:capsule, a2, b2, r2}) do
    {_closest1, _closest2, distance} = closest_points_segments(a1, b1, a2, b2)
    sum_radii = r1 + r2

    if distance < sum_radii do
      {:collision, sum_radii - distance}
    else
      :no_collision
    end
  end

  # ============================================================================
  # Sphere-Capsule Collision
  # ============================================================================

  @doc """
  Test collision between a sphere and a capsule.

  A sphere and capsule collide if the closest distance from the sphere's
  centre to the capsule's line segment is less than the sum of their radii.
  """
  @spec sphere_capsule(sphere(), capsule()) :: collision_result()
  def sphere_capsule({:sphere, centre, r_sphere}, {:capsule, cap_a, cap_b, r_capsule}) do
    {_closest, distance} = closest_point_on_segment(centre, cap_a, cap_b)
    sum_radii = r_sphere + r_capsule

    if distance < sum_radii do
      {:collision, sum_radii - distance}
    else
      :no_collision
    end
  end

  # ============================================================================
  # Sphere-Box (OBB) Collision
  # ============================================================================

  @doc """
  Test collision between a sphere and an oriented bounding box.

  The sphere collides with the box if the closest point on the box
  to the sphere's centre is within the sphere's radius.
  """
  @spec sphere_box(sphere(), box()) :: collision_result()
  def sphere_box({:sphere, centre, radius}, {:box, box_centre, half_extents, axes}) do
    {_closest, distance} = closest_point_on_box(centre, box_centre, half_extents, axes)

    if distance < radius do
      {:collision, radius - distance}
    else
      :no_collision
    end
  end

  # ============================================================================
  # Capsule-Box Collision
  # ============================================================================

  @doc """
  Test collision between a capsule and an oriented bounding box.

  Finds the closest distance between the capsule's line segment and the box,
  then checks if it's less than the capsule's radius.
  """
  @spec capsule_box(capsule(), box()) :: collision_result()
  def capsule_box({:capsule, cap_a, cap_b, radius}, {:box, box_centre, half_extents, axes}) do
    distance = closest_distance_segment_box(cap_a, cap_b, box_centre, half_extents, axes)

    if distance < radius do
      {:collision, radius - distance}
    else
      :no_collision
    end
  end

  # ============================================================================
  # Box-Box (OBB) Collision using Separating Axis Theorem
  # ============================================================================

  @doc """
  Test collision between two oriented bounding boxes using the Separating Axis Theorem.

  Two convex shapes are separated if there exists an axis along which their
  projections don't overlap. For two OBBs, we need to test 15 potential
  separating axes:
  - 3 face normals from box A
  - 3 face normals from box B
  - 9 cross products of edges from A and B
  """
  @spec box_box(box(), box()) :: collision_result()
  def box_box({:box, c1, h1, {a1x, a1y, a1z}}, {:box, c2, h2, {a2x, a2y, a2z}}) do
    # Vector from centre of box1 to centre of box2
    t = Vec3.subtract(c2, c1)

    # Half extents as floats
    {h1x, h1y, h1z} = {Vec3.x(h1), Vec3.y(h1), Vec3.z(h1)}
    {h2x, h2y, h2z} = {Vec3.x(h2), Vec3.y(h2), Vec3.z(h2)}

    # All axes to test
    axes = [
      # Face normals of box 1
      a1x,
      a1y,
      a1z,
      # Face normals of box 2
      a2x,
      a2y,
      a2z,
      # Cross products of edges
      Vec3.cross(a1x, a2x),
      Vec3.cross(a1x, a2y),
      Vec3.cross(a1x, a2z),
      Vec3.cross(a1y, a2x),
      Vec3.cross(a1y, a2y),
      Vec3.cross(a1y, a2z),
      Vec3.cross(a1z, a2x),
      Vec3.cross(a1z, a2y),
      Vec3.cross(a1z, a2z)
    ]

    box1_axes = {a1x, a1y, a1z}
    box2_axes = {a2x, a2y, a2z}
    box1_half = {h1x, h1y, h1z}
    box2_half = {h2x, h2y, h2z}

    # Find minimum penetration across all axes
    min_penetration =
      axes
      |> Enum.reduce_while(:infinity, fn axis, min_pen ->
        # Skip degenerate axes (from parallel edges)
        if Vec3.magnitude_squared(axis) < 1.0e-10 do
          {:cont, min_pen}
        else
          axis_normalized = Vec3.normalise(axis)

          case test_axis(axis_normalized, t, box1_axes, box1_half, box2_axes, box2_half) do
            :separated ->
              {:halt, :separated}

            {:overlap, pen} ->
              {:cont, min(min_pen, pen)}
          end
        end
      end)

    case min_penetration do
      :separated -> :no_collision
      :infinity -> :no_collision
      pen when is_float(pen) -> {:collision, pen}
    end
  end

  # ============================================================================
  # Helper Functions - Line Segment Operations
  # ============================================================================

  @doc """
  Find the closest point on a line segment to a given point.

  Returns `{closest_point, distance}`.
  """
  @spec closest_point_on_segment(Vec3.t(), Vec3.t(), Vec3.t()) :: {Vec3.t(), float()}
  def closest_point_on_segment(point, seg_a, seg_b) do
    ab = Vec3.subtract(seg_b, seg_a)
    ap = Vec3.subtract(point, seg_a)

    ab_len_sq = Vec3.magnitude_squared(ab)

    # Degenerate segment (point)
    if ab_len_sq < 1.0e-10 do
      {seg_a, Vec3.distance(point, seg_a)}
    else
      t = Vec3.dot(ap, ab) / ab_len_sq
      t_clamped = max(0.0, min(1.0, t))

      closest = Vec3.add(seg_a, Vec3.scale(ab, t_clamped))
      {closest, Vec3.distance(point, closest)}
    end
  end

  @doc """
  Find the closest points between two line segments.

  Returns `{closest_on_seg1, closest_on_seg2, distance}`.

  Uses the algorithm from "Real-Time Collision Detection" by Christer Ericson.
  """
  @spec closest_points_segments(Vec3.t(), Vec3.t(), Vec3.t(), Vec3.t()) ::
          {Vec3.t(), Vec3.t(), float()}
  def closest_points_segments(a1, b1, a2, b2) do
    d1 = Vec3.subtract(b1, a1)
    d2 = Vec3.subtract(b2, a2)
    r = Vec3.subtract(a1, a2)

    a = Vec3.dot(d1, d1)
    e = Vec3.dot(d2, d2)
    f = Vec3.dot(d2, r)

    # Check for degenerate segments
    cond do
      a < 1.0e-10 and e < 1.0e-10 ->
        # Both segments are points
        {a1, a2, Vec3.distance(a1, a2)}

      a < 1.0e-10 ->
        # First segment is a point
        {closest, dist} = closest_point_on_segment(a1, a2, b2)
        {a1, closest, dist}

      e < 1.0e-10 ->
        # Second segment is a point
        {closest, dist} = closest_point_on_segment(a2, a1, b1)
        {closest, a2, dist}

      true ->
        # General case
        c = Vec3.dot(d1, r)
        b = Vec3.dot(d1, d2)
        denom = a * e - b * b

        # Compute s (parameter on first segment)
        s =
          if abs(denom) < 1.0e-10 do
            0.0
          else
            clamp((b * f - c * e) / denom, 0.0, 1.0)
          end

        # Compute t (parameter on second segment)
        t = (b * s + f) / e

        # Clamp t and recompute s if needed
        {s, t} =
          cond do
            t < 0.0 ->
              {clamp(-c / a, 0.0, 1.0), 0.0}

            t > 1.0 ->
              {clamp((b - c) / a, 0.0, 1.0), 1.0}

            true ->
              {s, t}
          end

        closest1 = Vec3.add(a1, Vec3.scale(d1, s))
        closest2 = Vec3.add(a2, Vec3.scale(d2, t))
        {closest1, closest2, Vec3.distance(closest1, closest2)}
    end
  end

  # ============================================================================
  # Helper Functions - Box Operations
  # ============================================================================

  @doc """
  Find the closest point on an OBB to a given point.

  Returns `{closest_point, distance}`.
  """
  @spec closest_point_on_box(Vec3.t(), Vec3.t(), Vec3.t(), {Vec3.t(), Vec3.t(), Vec3.t()}) ::
          {Vec3.t(), float()}
  def closest_point_on_box(point, box_centre, half_extents, {ax, ay, az}) do
    # Vector from box centre to point
    d = Vec3.subtract(point, box_centre)

    # Project onto each axis and clamp
    {hx, hy, hz} = {Vec3.x(half_extents), Vec3.y(half_extents), Vec3.z(half_extents)}

    dx = clamp(Vec3.dot(d, ax), -hx, hx)
    dy = clamp(Vec3.dot(d, ay), -hy, hy)
    dz = clamp(Vec3.dot(d, az), -hz, hz)

    # Reconstruct closest point
    closest =
      box_centre
      |> Vec3.add(Vec3.scale(ax, dx))
      |> Vec3.add(Vec3.scale(ay, dy))
      |> Vec3.add(Vec3.scale(az, dz))

    {closest, Vec3.distance(point, closest)}
  end

  # ============================================================================
  # Helper Functions - Capsule-Box
  # ============================================================================

  # Find the closest distance between a line segment and an OBB
  defp closest_distance_segment_box(seg_a, seg_b, box_centre, half_extents, axes) do
    # Sample points along the segment and find minimum distance to box
    # This is an approximation - exact solution is more complex
    num_samples = 8

    0..num_samples
    |> Enum.map(fn i ->
      t = i / num_samples
      point = Vec3.lerp(seg_a, seg_b, t)
      {_closest, distance} = closest_point_on_box(point, box_centre, half_extents, axes)
      distance
    end)
    |> Enum.min()
  end

  # ============================================================================
  # Helper Functions - SAT (Separating Axis Theorem)
  # ============================================================================

  # Test a single axis for the SAT algorithm
  defp test_axis(axis, t, {a1x, a1y, a1z}, {h1x, h1y, h1z}, {a2x, a2y, a2z}, {h2x, h2y, h2z}) do
    # Project the translation vector onto the axis
    t_proj = abs(Vec3.dot(t, axis))

    # Project box 1's half-extents onto the axis
    r1 =
      h1x * abs(Vec3.dot(a1x, axis)) +
        h1y * abs(Vec3.dot(a1y, axis)) +
        h1z * abs(Vec3.dot(a1z, axis))

    # Project box 2's half-extents onto the axis
    r2 =
      h2x * abs(Vec3.dot(a2x, axis)) +
        h2y * abs(Vec3.dot(a2y, axis)) +
        h2z * abs(Vec3.dot(a2z, axis))

    # Check for separation
    if t_proj > r1 + r2 do
      :separated
    else
      {:overlap, r1 + r2 - t_proj}
    end
  end

  # ============================================================================
  # Helper Functions - Geometry Expansion
  # ============================================================================

  defp expand_geometry({:sphere, centre, radius}, margin) do
    {:sphere, centre, radius + margin}
  end

  defp expand_geometry({:capsule, a, b, radius}, margin) do
    {:capsule, a, b, radius + margin}
  end

  defp expand_geometry({:box, centre, half_extents, axes}, margin) do
    # Expand each half-extent by the margin
    expanded =
      Vec3.new(
        Vec3.x(half_extents) + margin,
        Vec3.y(half_extents) + margin,
        Vec3.z(half_extents) + margin
      )

    {:box, centre, expanded, axes}
  end

  # ============================================================================
  # Utility Functions
  # ============================================================================

  defp clamp(value, min_val, max_val) do
    value |> max(min_val) |> min(max_val)
  end
end
