# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision.PrimitivesTest do
  use ExUnit.Case, async: true

  alias BB.Collision.Primitives
  alias BB.Math.Vec3

  describe "sphere_sphere/2" do
    test "detects collision between overlapping spheres" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(1.5, 0.0, 0.0), 1.0}

      assert {:collision, penetration} = Primitives.sphere_sphere(sphere1, sphere2)
      assert_in_delta penetration, 0.5, 0.001
    end

    test "detects collision between touching spheres" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(2.0, 0.0, 0.0), 1.0}

      # Exactly touching - not technically colliding
      assert :no_collision = Primitives.sphere_sphere(sphere1, sphere2)
    end

    test "returns no collision for separated spheres" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(3.0, 0.0, 0.0), 1.0}

      assert :no_collision = Primitives.sphere_sphere(sphere1, sphere2)
    end

    test "detects collision between concentric spheres" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 2.0}
      sphere2 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}

      assert {:collision, penetration} = Primitives.sphere_sphere(sphere1, sphere2)
      assert_in_delta penetration, 3.0, 0.001
    end

    test "works with different radii" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 0.5}
      sphere2 = {:sphere, Vec3.new(1.0, 0.0, 0.0), 1.0}

      assert {:collision, penetration} = Primitives.sphere_sphere(sphere1, sphere2)
      assert_in_delta penetration, 0.5, 0.001
    end
  end

  describe "capsule_capsule/2" do
    test "detects collision between parallel capsules" do
      # Two horizontal capsules side by side
      cap1 = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.5}
      cap2 = {:capsule, Vec3.new(0.0, 0.8, 0.0), Vec3.new(2.0, 0.8, 0.0), 0.5}

      assert {:collision, penetration} = Primitives.capsule_capsule(cap1, cap2)
      assert_in_delta penetration, 0.2, 0.001
    end

    test "detects collision between crossing capsules" do
      # One horizontal, one vertical, crossing in the middle
      cap1 = {:capsule, Vec3.new(-1.0, 0.0, 0.0), Vec3.new(1.0, 0.0, 0.0), 0.3}
      cap2 = {:capsule, Vec3.new(0.0, -1.0, 0.0), Vec3.new(0.0, 1.0, 0.0), 0.3}

      assert {:collision, penetration} = Primitives.capsule_capsule(cap1, cap2)
      assert_in_delta penetration, 0.6, 0.001
    end

    test "returns no collision for separated capsules" do
      cap1 = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.3}
      cap2 = {:capsule, Vec3.new(0.0, 2.0, 0.0), Vec3.new(2.0, 2.0, 0.0), 0.3}

      assert :no_collision = Primitives.capsule_capsule(cap1, cap2)
    end

    test "detects end-to-end collision" do
      # Capsules touching at their ends
      cap1 = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 0.0, 0.0), 0.5}
      cap2 = {:capsule, Vec3.new(1.5, 0.0, 0.0), Vec3.new(2.5, 0.0, 0.0), 0.5}

      assert {:collision, penetration} = Primitives.capsule_capsule(cap1, cap2)
      assert_in_delta penetration, 0.5, 0.001
    end

    test "handles degenerate capsules (points)" do
      # Capsule with zero length is essentially a sphere
      cap1 = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(0.0, 0.0, 0.0), 0.5}
      cap2 = {:capsule, Vec3.new(0.8, 0.0, 0.0), Vec3.new(0.8, 0.0, 0.0), 0.5}

      assert {:collision, penetration} = Primitives.capsule_capsule(cap1, cap2)
      assert_in_delta penetration, 0.2, 0.001
    end
  end

  describe "sphere_capsule/2" do
    test "detects collision when sphere hits middle of capsule" do
      sphere = {:sphere, Vec3.new(1.0, 0.6, 0.0), 0.5}
      capsule = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.3}

      assert {:collision, penetration} = Primitives.sphere_capsule(sphere, capsule)
      assert_in_delta penetration, 0.2, 0.001
    end

    test "detects collision at capsule end" do
      sphere = {:sphere, Vec3.new(2.3, 0.0, 0.0), 0.5}
      capsule = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.3}

      assert {:collision, penetration} = Primitives.sphere_capsule(sphere, capsule)
      assert_in_delta penetration, 0.5, 0.001
    end

    test "returns no collision for separated sphere and capsule" do
      sphere = {:sphere, Vec3.new(1.0, 2.0, 0.0), 0.3}
      capsule = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.3}

      assert :no_collision = Primitives.sphere_capsule(sphere, capsule)
    end
  end

  describe "sphere_box/2" do
    test "detects collision when sphere inside box" do
      sphere = {:sphere, Vec3.new(0.0, 0.0, 0.0), 0.3}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert {:collision, _penetration} = Primitives.sphere_box(sphere, box)
    end

    test "detects collision when sphere touches box face" do
      sphere = {:sphere, Vec3.new(1.3, 0.0, 0.0), 0.5}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert {:collision, penetration} = Primitives.sphere_box(sphere, box)
      assert_in_delta penetration, 0.2, 0.001
    end

    test "detects collision when sphere touches box corner" do
      # Sphere near corner of unit box centred at origin
      # Distance from sphere centre (1.3, 1.3, 1.3) to box corner (1, 1, 1) is sqrt(0.3^2 * 3) ≈ 0.52
      # Sphere radius is 0.6, so should collide
      sphere = {:sphere, Vec3.new(1.0 + 0.3, 1.0 + 0.3, 1.0 + 0.3), 0.6}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert {:collision, _penetration} = Primitives.sphere_box(sphere, box)
    end

    test "returns no collision for separated sphere and box" do
      sphere = {:sphere, Vec3.new(3.0, 0.0, 0.0), 0.5}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert :no_collision = Primitives.sphere_box(sphere, box)
    end

    test "works with rotated box" do
      # Box rotated 45 degrees around Z axis
      # Half-extent of 0.5 along rotated axis extends to sqrt(2)/2 * 0.5 ≈ 0.354 in each direction
      # So box corner reaches about 0.707 in X direction
      angle = :math.pi() / 4
      cos_a = :math.cos(angle)
      sin_a = :math.sin(angle)

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(0.5, 0.5, 0.5),
         {Vec3.new(cos_a, sin_a, 0.0), Vec3.new(-sin_a, cos_a, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      # Sphere far enough away to not collide
      sphere_far = {:sphere, Vec3.new(1.5, 0.0, 0.0), 0.3}
      assert :no_collision = Primitives.sphere_box(sphere_far, box)

      # Sphere close enough to collide with rotated corner
      sphere_near = {:sphere, Vec3.new(0.9, 0.0, 0.0), 0.3}
      assert {:collision, _} = Primitives.sphere_box(sphere_near, box)
    end
  end

  describe "capsule_box/2" do
    test "detects collision when capsule passes through box" do
      capsule = {:capsule, Vec3.new(-2.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.2}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert {:collision, _penetration} = Primitives.capsule_box(capsule, box)
    end

    test "returns no collision for separated capsule and box" do
      capsule = {:capsule, Vec3.new(0.0, 3.0, 0.0), Vec3.new(2.0, 3.0, 0.0), 0.2}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert :no_collision = Primitives.capsule_box(capsule, box)
    end

    test "detects collision when capsule grazes box edge" do
      capsule = {:capsule, Vec3.new(1.2, -1.0, 0.0), Vec3.new(1.2, 1.0, 0.0), 0.3}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      # Capsule is 0.2 units from box edge, radius is 0.3
      assert {:collision, penetration} = Primitives.capsule_box(capsule, box)
      assert_in_delta penetration, 0.1, 0.05
    end
  end

  describe "box_box/2" do
    test "detects collision between overlapping axis-aligned boxes" do
      box1 =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      box2 =
        {:box, Vec3.new(1.5, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert {:collision, penetration} = Primitives.box_box(box1, box2)
      assert_in_delta penetration, 0.5, 0.001
    end

    test "returns no collision for separated axis-aligned boxes" do
      box1 =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      box2 =
        {:box, Vec3.new(3.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert :no_collision = Primitives.box_box(box1, box2)
    end

    test "detects collision between rotated boxes" do
      box1 =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      # Box rotated 45 degrees around Z
      angle = :math.pi() / 4
      cos_a = :math.cos(angle)
      sin_a = :math.sin(angle)

      box2 =
        {:box, Vec3.new(1.5, 0.0, 0.0), Vec3.new(0.5, 0.5, 0.5),
         {Vec3.new(cos_a, sin_a, 0.0), Vec3.new(-sin_a, cos_a, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      assert {:collision, _penetration} = Primitives.box_box(box1, box2)
    end

    test "correctly handles edge-edge separation" do
      # Two boxes that would collide on face normals but are separated on edge cross products
      angle = :math.pi() / 4
      cos_a = :math.cos(angle)
      sin_a = :math.sin(angle)

      box1 =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(0.5, 0.5, 0.5),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      box2 =
        {:box, Vec3.new(1.5, 1.5, 0.0), Vec3.new(0.5, 0.5, 0.5),
         {Vec3.new(cos_a, sin_a, 0.0), Vec3.new(-sin_a, cos_a, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      # These boxes should not collide
      assert :no_collision = Primitives.box_box(box1, box2)
    end
  end

  describe "test/2 dispatch" do
    test "correctly dispatches sphere-sphere" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(1.5, 0.0, 0.0), 1.0}

      assert {:collision, _} = Primitives.test(sphere1, sphere2)
    end

    test "correctly dispatches capsule-capsule" do
      cap1 = {:capsule, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 0.0, 0.0), 0.5}
      cap2 = {:capsule, Vec3.new(0.5, 0.8, 0.0), Vec3.new(1.5, 0.8, 0.0), 0.5}

      assert {:collision, _} = Primitives.test(cap1, cap2)
    end

    test "is symmetric for mixed types" do
      sphere = {:sphere, Vec3.new(0.0, 0.0, 0.0), 0.5}
      capsule = {:capsule, Vec3.new(0.0, 0.6, 0.0), Vec3.new(1.0, 0.6, 0.0), 0.3}

      result1 = Primitives.test(sphere, capsule)
      result2 = Primitives.test(capsule, sphere)

      assert result1 == result2
    end

    test "is symmetric for sphere-box" do
      sphere = {:sphere, Vec3.new(1.3, 0.0, 0.0), 0.5}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      result1 = Primitives.test(sphere, box)
      result2 = Primitives.test(box, sphere)

      assert result1 == result2
    end

    test "is symmetric for capsule-box" do
      capsule = {:capsule, Vec3.new(-2.0, 0.0, 0.0), Vec3.new(2.0, 0.0, 0.0), 0.2}

      box =
        {:box, Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0),
         {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}}

      result1 = Primitives.test(capsule, box)
      result2 = Primitives.test(box, capsule)

      assert result1 == result2
    end
  end

  describe "test_with_margin/3" do
    test "detects near-misses with margin" do
      # Two spheres that are close but not touching
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(2.1, 0.0, 0.0), 1.0}

      # Without margin, no collision
      assert :no_collision = Primitives.test(sphere1, sphere2)

      # With 0.1 margin, should detect collision
      assert {:collision, _} = Primitives.test_with_margin(sphere1, sphere2, 0.1)
    end

    test "margin of zero behaves like test/2" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(1.5, 0.0, 0.0), 1.0}

      assert Primitives.test(sphere1, sphere2) == Primitives.test_with_margin(sphere1, sphere2, 0)
    end

    test "negative margin behaves like test/2" do
      sphere1 = {:sphere, Vec3.new(0.0, 0.0, 0.0), 1.0}
      sphere2 = {:sphere, Vec3.new(1.5, 0.0, 0.0), 1.0}

      assert Primitives.test(sphere1, sphere2) ==
               Primitives.test_with_margin(sphere1, sphere2, -0.1)
    end
  end

  describe "closest_point_on_segment/3" do
    test "returns endpoint when point is before segment" do
      {closest, _distance} =
        Primitives.closest_point_on_segment(
          Vec3.new(-1.0, 0.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(2.0, 0.0, 0.0)
        )

      assert_in_delta Vec3.x(closest), 0.0, 0.001
      assert_in_delta Vec3.y(closest), 0.0, 0.001
      assert_in_delta Vec3.z(closest), 0.0, 0.001
    end

    test "returns endpoint when point is after segment" do
      {closest, _distance} =
        Primitives.closest_point_on_segment(
          Vec3.new(3.0, 0.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(2.0, 0.0, 0.0)
        )

      assert_in_delta Vec3.x(closest), 2.0, 0.001
      assert_in_delta Vec3.y(closest), 0.0, 0.001
      assert_in_delta Vec3.z(closest), 0.0, 0.001
    end

    test "returns point on segment when point is beside it" do
      {closest, distance} =
        Primitives.closest_point_on_segment(
          Vec3.new(1.0, 1.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(2.0, 0.0, 0.0)
        )

      assert_in_delta Vec3.x(closest), 1.0, 0.001
      assert_in_delta Vec3.y(closest), 0.0, 0.001
      assert_in_delta distance, 1.0, 0.001
    end

    test "handles degenerate segment" do
      {closest, distance} =
        Primitives.closest_point_on_segment(
          Vec3.new(1.0, 1.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0)
        )

      assert_in_delta Vec3.x(closest), 0.0, 0.001
      assert_in_delta distance, :math.sqrt(2), 0.001
    end
  end

  describe "closest_points_segments/4" do
    test "finds closest points on parallel segments" do
      {c1, c2, distance} =
        Primitives.closest_points_segments(
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(2.0, 0.0, 0.0),
          Vec3.new(0.0, 1.0, 0.0),
          Vec3.new(2.0, 1.0, 0.0)
        )

      # Any pair of points at same X should work, distance is 1
      assert_in_delta Vec3.y(c1), 0.0, 0.001
      assert_in_delta Vec3.y(c2), 1.0, 0.001
      assert_in_delta distance, 1.0, 0.001
    end

    test "finds closest points on crossing segments" do
      {c1, c2, distance} =
        Primitives.closest_points_segments(
          Vec3.new(-1.0, 0.0, 0.0),
          Vec3.new(1.0, 0.0, 0.0),
          Vec3.new(0.0, -1.0, 1.0),
          Vec3.new(0.0, 1.0, 1.0)
        )

      # Closest points should be at origin of each segment's projection
      assert_in_delta Vec3.x(c1), 0.0, 0.001
      assert_in_delta Vec3.y(c1), 0.0, 0.001
      assert_in_delta Vec3.z(c1), 0.0, 0.001
      assert_in_delta Vec3.x(c2), 0.0, 0.001
      assert_in_delta Vec3.y(c2), 0.0, 0.001
      assert_in_delta Vec3.z(c2), 1.0, 0.001
      assert_in_delta distance, 1.0, 0.001
    end

    test "handles end-to-end segments" do
      {_c1, _c2, distance} =
        Primitives.closest_points_segments(
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(1.0, 0.0, 0.0),
          Vec3.new(1.5, 0.0, 0.0),
          Vec3.new(2.5, 0.0, 0.0)
        )

      assert_in_delta distance, 0.5, 0.001
    end
  end

  describe "closest_point_on_box/4" do
    test "returns centre for point inside box" do
      {closest, distance} =
        Primitives.closest_point_on_box(
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(1.0, 1.0, 1.0),
          {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}
        )

      assert_in_delta Vec3.x(closest), 0.0, 0.001
      assert_in_delta Vec3.y(closest), 0.0, 0.001
      assert_in_delta Vec3.z(closest), 0.0, 0.001
      assert_in_delta distance, 0.0, 0.001
    end

    test "returns face point for point outside face" do
      {closest, distance} =
        Primitives.closest_point_on_box(
          Vec3.new(2.0, 0.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(1.0, 1.0, 1.0),
          {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}
        )

      assert_in_delta Vec3.x(closest), 1.0, 0.001
      assert_in_delta Vec3.y(closest), 0.0, 0.001
      assert_in_delta Vec3.z(closest), 0.0, 0.001
      assert_in_delta distance, 1.0, 0.001
    end

    test "returns corner point for point outside corner" do
      {closest, distance} =
        Primitives.closest_point_on_box(
          Vec3.new(2.0, 2.0, 2.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(1.0, 1.0, 1.0),
          {Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 1.0, 0.0), Vec3.new(0.0, 0.0, 1.0)}
        )

      assert_in_delta Vec3.x(closest), 1.0, 0.001
      assert_in_delta Vec3.y(closest), 1.0, 0.001
      assert_in_delta Vec3.z(closest), 1.0, 0.001
      assert_in_delta distance, :math.sqrt(3), 0.001
    end

    test "works with rotated box" do
      angle = :math.pi() / 4
      cos_a = :math.cos(angle)
      sin_a = :math.sin(angle)

      {_closest, distance} =
        Primitives.closest_point_on_box(
          Vec3.new(2.0, 0.0, 0.0),
          Vec3.new(0.0, 0.0, 0.0),
          Vec3.new(1.0, 1.0, 1.0),
          {Vec3.new(cos_a, sin_a, 0.0), Vec3.new(-sin_a, cos_a, 0.0), Vec3.new(0.0, 0.0, 1.0)}
        )

      # Rotated box extends further in X direction
      # Distance should be less than 1.0
      assert distance < 1.0
    end
  end
end
