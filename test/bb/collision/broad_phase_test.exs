# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision.BroadPhaseTest do
  use ExUnit.Case, async: true

  alias BB.Collision.BroadPhase
  alias BB.Math.{Transform, Vec3, Quaternion}

  describe "overlap?/2" do
    test "detects overlapping AABBs" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 2.0, 2.0)}
      aabb2 = {Vec3.new(1.0, 1.0, 1.0), Vec3.new(3.0, 3.0, 3.0)}

      assert BroadPhase.overlap?(aabb1, aabb2)
    end

    test "detects touching AABBs as overlapping" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}
      aabb2 = {Vec3.new(1.0, 0.0, 0.0), Vec3.new(2.0, 1.0, 1.0)}

      assert BroadPhase.overlap?(aabb1, aabb2)
    end

    test "returns false for separated AABBs on X axis" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}
      aabb2 = {Vec3.new(2.0, 0.0, 0.0), Vec3.new(3.0, 1.0, 1.0)}

      refute BroadPhase.overlap?(aabb1, aabb2)
    end

    test "returns false for separated AABBs on Y axis" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}
      aabb2 = {Vec3.new(0.0, 2.0, 0.0), Vec3.new(1.0, 3.0, 1.0)}

      refute BroadPhase.overlap?(aabb1, aabb2)
    end

    test "returns false for separated AABBs on Z axis" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}
      aabb2 = {Vec3.new(0.0, 0.0, 2.0), Vec3.new(1.0, 1.0, 3.0)}

      refute BroadPhase.overlap?(aabb1, aabb2)
    end

    test "detects one AABB containing another" do
      outer = {Vec3.new(-1.0, -1.0, -1.0), Vec3.new(3.0, 3.0, 3.0)}
      inner = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}

      assert BroadPhase.overlap?(outer, inner)
      assert BroadPhase.overlap?(inner, outer)
    end
  end

  describe "compute_aabb/2 for spheres" do
    test "sphere at origin" do
      geometry = {:sphere, %{radius: 1.0}}
      transform = Transform.identity()

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), -1.0, 0.001
      assert_in_delta Vec3.y(min_pt), -1.0, 0.001
      assert_in_delta Vec3.z(min_pt), -1.0, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 1.0, 0.001
      assert_in_delta Vec3.z(max_pt), 1.0, 0.001
    end

    test "sphere with translation" do
      geometry = {:sphere, %{radius: 0.5}}
      transform = Transform.from_position_quaternion(Vec3.new(1.0, 2.0, 3.0), Quaternion.identity())

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), 0.5, 0.001
      assert_in_delta Vec3.y(min_pt), 1.5, 0.001
      assert_in_delta Vec3.z(min_pt), 2.5, 0.001
      assert_in_delta Vec3.x(max_pt), 1.5, 0.001
      assert_in_delta Vec3.y(max_pt), 2.5, 0.001
      assert_in_delta Vec3.z(max_pt), 3.5, 0.001
    end

    test "sphere rotation has no effect" do
      geometry = {:sphere, %{radius: 1.0}}
      rotation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 4)
      transform = Transform.from_quaternion(rotation)

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), -1.0, 0.001
      assert_in_delta Vec3.y(min_pt), -1.0, 0.001
      assert_in_delta Vec3.z(min_pt), -1.0, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 1.0, 0.001
      assert_in_delta Vec3.z(max_pt), 1.0, 0.001
    end
  end

  describe "compute_aabb/2 for capsules" do
    test "capsule aligned with Z axis at origin" do
      geometry = {:capsule, %{radius: 0.5, length: 2.0}}
      transform = Transform.identity()

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # Capsule extends from z=-1 to z=1 (half length), plus radius
      assert_in_delta Vec3.x(min_pt), -0.5, 0.001
      assert_in_delta Vec3.y(min_pt), -0.5, 0.001
      assert_in_delta Vec3.z(min_pt), -1.5, 0.001
      assert_in_delta Vec3.x(max_pt), 0.5, 0.001
      assert_in_delta Vec3.y(max_pt), 0.5, 0.001
      assert_in_delta Vec3.z(max_pt), 1.5, 0.001
    end

    test "capsule rotated 90 degrees around Y" do
      geometry = {:capsule, %{radius: 0.5, length: 2.0}}
      rotation = Quaternion.from_axis_angle(Vec3.unit_y(), :math.pi() / 2)
      transform = Transform.from_quaternion(rotation)

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # After rotation, capsule lies along X axis
      assert_in_delta Vec3.x(min_pt), -1.5, 0.001
      assert_in_delta Vec3.y(min_pt), -0.5, 0.001
      assert_in_delta Vec3.z(min_pt), -0.5, 0.001
      assert_in_delta Vec3.x(max_pt), 1.5, 0.001
      assert_in_delta Vec3.y(max_pt), 0.5, 0.001
      assert_in_delta Vec3.z(max_pt), 0.5, 0.001
    end

    test "capsule with translation" do
      geometry = {:capsule, %{radius: 0.5, length: 2.0}}
      transform = Transform.from_position_quaternion(Vec3.new(5.0, 0.0, 0.0), Quaternion.identity())

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), 4.5, 0.001
      assert_in_delta Vec3.x(max_pt), 5.5, 0.001
    end
  end

  describe "compute_aabb/2 for cylinders" do
    test "cylinder treated as capsule" do
      capsule_geometry = {:capsule, %{radius: 0.5, length: 2.0}}
      cylinder_geometry = {:cylinder, %{radius: 0.5, height: 2.0}}
      transform = Transform.identity()

      capsule_aabb = BroadPhase.compute_aabb(capsule_geometry, transform)
      cylinder_aabb = BroadPhase.compute_aabb(cylinder_geometry, transform)

      {cap_min, cap_max} = capsule_aabb
      {cyl_min, cyl_max} = cylinder_aabb

      assert_in_delta Vec3.x(cap_min), Vec3.x(cyl_min), 0.001
      assert_in_delta Vec3.y(cap_min), Vec3.y(cyl_min), 0.001
      assert_in_delta Vec3.z(cap_min), Vec3.z(cyl_min), 0.001
      assert_in_delta Vec3.x(cap_max), Vec3.x(cyl_max), 0.001
      assert_in_delta Vec3.y(cap_max), Vec3.y(cyl_max), 0.001
      assert_in_delta Vec3.z(cap_max), Vec3.z(cyl_max), 0.001
    end
  end

  describe "compute_aabb/2 for boxes" do
    test "axis-aligned box at origin" do
      geometry = {:box, %{x: 1.0, y: 2.0, z: 0.5}}
      transform = Transform.identity()

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), -1.0, 0.001
      assert_in_delta Vec3.y(min_pt), -2.0, 0.001
      assert_in_delta Vec3.z(min_pt), -0.5, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 2.0, 0.001
      assert_in_delta Vec3.z(max_pt), 0.5, 0.001
    end

    test "box rotated 45 degrees around Z" do
      geometry = {:box, %{x: 1.0, y: 1.0, z: 1.0}}
      rotation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 4)
      transform = Transform.from_quaternion(rotation)

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # When rotated 45 degrees, the AABB expands
      # sqrt(2) ≈ 1.414
      expected_extent = :math.sqrt(2)
      assert_in_delta Vec3.x(min_pt), -expected_extent, 0.001
      assert_in_delta Vec3.y(min_pt), -expected_extent, 0.001
      assert_in_delta Vec3.z(min_pt), -1.0, 0.001
      assert_in_delta Vec3.x(max_pt), expected_extent, 0.001
      assert_in_delta Vec3.y(max_pt), expected_extent, 0.001
      assert_in_delta Vec3.z(max_pt), 1.0, 0.001
    end

    test "box with translation" do
      geometry = {:box, %{x: 0.5, y: 0.5, z: 0.5}}
      transform = Transform.from_position_quaternion(Vec3.new(10.0, 20.0, 30.0), Quaternion.identity())

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), 9.5, 0.001
      assert_in_delta Vec3.y(min_pt), 19.5, 0.001
      assert_in_delta Vec3.z(min_pt), 29.5, 0.001
      assert_in_delta Vec3.x(max_pt), 10.5, 0.001
      assert_in_delta Vec3.y(max_pt), 20.5, 0.001
      assert_in_delta Vec3.z(max_pt), 30.5, 0.001
    end
  end

  describe "compute_aabb/2 for meshes" do
    @fixtures_dir Path.join([__DIR__, "..", "..", "fixtures"])

    test "mesh with valid file computes accurate AABB" do
      path = Path.join(@fixtures_dir, "cube.stl")
      geometry = {:mesh, %{filename: path, scale: 1.0}}
      transform = Transform.identity()

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # Cube is 0-1 in all dimensions
      assert_in_delta Vec3.x(min_pt), 0.0, 0.001
      assert_in_delta Vec3.y(min_pt), 0.0, 0.001
      assert_in_delta Vec3.z(min_pt), 0.0, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 1.0, 0.001
      assert_in_delta Vec3.z(max_pt), 1.0, 0.001
    end

    test "mesh with scale applies scaling" do
      path = Path.join(@fixtures_dir, "cube.stl")
      geometry = {:mesh, %{filename: path, scale: 2.0}}
      transform = Transform.identity()

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # Scaled cube is 0-2 in all dimensions
      assert_in_delta Vec3.x(min_pt), 0.0, 0.001
      assert_in_delta Vec3.y(min_pt), 0.0, 0.001
      assert_in_delta Vec3.z(min_pt), 0.0, 0.001
      assert_in_delta Vec3.x(max_pt), 2.0, 0.001
      assert_in_delta Vec3.y(max_pt), 2.0, 0.001
      assert_in_delta Vec3.z(max_pt), 2.0, 0.001
    end

    test "mesh with translation" do
      path = Path.join(@fixtures_dir, "cube.stl")
      geometry = {:mesh, %{filename: path, scale: 1.0}}
      transform = Transform.from_position_quaternion(Vec3.new(5.0, 0.0, 0.0), Quaternion.identity())

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      assert_in_delta Vec3.x(min_pt), 5.0, 0.001
      assert_in_delta Vec3.x(max_pt), 6.0, 0.001
    end

    test "mesh with non-existent file returns placeholder AABB" do
      geometry = {:mesh, %{filename: "/nonexistent/model.stl", scale: 1.0}}
      transform = Transform.from_position_quaternion(Vec3.new(5.0, 0.0, 0.0), Quaternion.identity())

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # Placeholder is a unit sphere around the translation
      assert_in_delta Vec3.x(min_pt), 4.0, 0.001
      assert_in_delta Vec3.x(max_pt), 6.0, 0.001
    end

    test "mesh without filename returns placeholder AABB" do
      geometry = {:mesh, %{other_data: "something"}}
      transform = Transform.from_position_quaternion(Vec3.new(5.0, 0.0, 0.0), Quaternion.identity())

      {min_pt, max_pt} = BroadPhase.compute_aabb(geometry, transform)

      # Placeholder is a unit sphere around the translation
      assert_in_delta Vec3.x(min_pt), 4.0, 0.001
      assert_in_delta Vec3.x(max_pt), 6.0, 0.001
    end
  end

  describe "expand/2" do
    test "expands AABB by margin" do
      aabb = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}

      {min_pt, max_pt} = BroadPhase.expand(aabb, 0.5)

      assert_in_delta Vec3.x(min_pt), -0.5, 0.001
      assert_in_delta Vec3.y(min_pt), -0.5, 0.001
      assert_in_delta Vec3.z(min_pt), -0.5, 0.001
      assert_in_delta Vec3.x(max_pt), 1.5, 0.001
      assert_in_delta Vec3.y(max_pt), 1.5, 0.001
      assert_in_delta Vec3.z(max_pt), 1.5, 0.001
    end
  end

  describe "merge/2" do
    test "merges two AABBs" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}
      aabb2 = {Vec3.new(2.0, 2.0, 2.0), Vec3.new(3.0, 3.0, 3.0)}

      {min_pt, max_pt} = BroadPhase.merge(aabb1, aabb2)

      assert_in_delta Vec3.x(min_pt), 0.0, 0.001
      assert_in_delta Vec3.y(min_pt), 0.0, 0.001
      assert_in_delta Vec3.z(min_pt), 0.0, 0.001
      assert_in_delta Vec3.x(max_pt), 3.0, 0.001
      assert_in_delta Vec3.y(max_pt), 3.0, 0.001
      assert_in_delta Vec3.z(max_pt), 3.0, 0.001
    end

    test "merge is commutative" do
      aabb1 = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(1.0, 1.0, 1.0)}
      aabb2 = {Vec3.new(-1.0, 2.0, 0.5), Vec3.new(0.5, 3.0, 2.0)}

      result1 = BroadPhase.merge(aabb1, aabb2)
      result2 = BroadPhase.merge(aabb2, aabb1)

      {min1, max1} = result1
      {min2, max2} = result2

      assert_in_delta Vec3.x(min1), Vec3.x(min2), 0.001
      assert_in_delta Vec3.y(min1), Vec3.y(min2), 0.001
      assert_in_delta Vec3.z(min1), Vec3.z(min2), 0.001
      assert_in_delta Vec3.x(max1), Vec3.x(max2), 0.001
      assert_in_delta Vec3.y(max1), Vec3.y(max2), 0.001
      assert_in_delta Vec3.z(max1), Vec3.z(max2), 0.001
    end
  end

  describe "centre/1" do
    test "computes centre of AABB" do
      aabb = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 4.0, 6.0)}

      centre = BroadPhase.centre(aabb)

      assert_in_delta Vec3.x(centre), 1.0, 0.001
      assert_in_delta Vec3.y(centre), 2.0, 0.001
      assert_in_delta Vec3.z(centre), 3.0, 0.001
    end
  end

  describe "size/1" do
    test "computes size of AABB" do
      aabb = {Vec3.new(1.0, 2.0, 3.0), Vec3.new(4.0, 6.0, 9.0)}

      size = BroadPhase.size(aabb)

      assert_in_delta Vec3.x(size), 3.0, 0.001
      assert_in_delta Vec3.y(size), 4.0, 0.001
      assert_in_delta Vec3.z(size), 6.0, 0.001
    end
  end

  describe "contains_point?/2" do
    test "point inside AABB" do
      aabb = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 2.0, 2.0)}
      point = Vec3.new(1.0, 1.0, 1.0)

      assert BroadPhase.contains_point?(aabb, point)
    end

    test "point on AABB boundary" do
      aabb = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 2.0, 2.0)}
      point = Vec3.new(0.0, 1.0, 1.0)

      assert BroadPhase.contains_point?(aabb, point)
    end

    test "point outside AABB" do
      aabb = {Vec3.new(0.0, 0.0, 0.0), Vec3.new(2.0, 2.0, 2.0)}
      point = Vec3.new(3.0, 1.0, 1.0)

      refute BroadPhase.contains_point?(aabb, point)
    end
  end
end
