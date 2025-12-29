# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.CollisionTest do
  use ExUnit.Case, async: true

  alias BB.Collision
  alias BB.ExampleRobots.CollisionTestArm
  alias BB.Math.{Quaternion, Vec3}

  describe "build_adjacency_set/1" do
    test "builds adjacency set from robot topology" do
      robot = CollisionTestArm.robot()
      adjacent = Collision.build_adjacency_set(robot)

      # Base and upper_arm are connected via shoulder joint
      assert MapSet.member?(adjacent, {:base, :upper_arm})
      assert MapSet.member?(adjacent, {:upper_arm, :base})

      # Upper_arm and forearm are connected via elbow joint
      assert MapSet.member?(adjacent, {:upper_arm, :forearm})
      assert MapSet.member?(adjacent, {:forearm, :upper_arm})

      # Base and forearm are NOT adjacent
      refute MapSet.member?(adjacent, {:base, :forearm})
      refute MapSet.member?(adjacent, {:forearm, :base})
    end
  end

  describe "self_collision?/3" do
    test "returns false when robot is in safe configuration" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      refute Collision.self_collision?(robot, positions)
    end

    test "returns true when robot is in self-collision with margin" do
      robot = CollisionTestArm.robot()
      # With a very large margin, the collision volumes expand enough to overlap
      positions = %{shoulder: 0.0, elbow: 0.0}

      # Without margin, no collision (links are well separated)
      refute Collision.self_collision?(robot, positions)

      # With very large margin (0.5m), base and forearm volumes will overlap
      assert Collision.self_collision?(robot, positions, margin: 0.5)
    end

    test "respects margin parameter" do
      robot = CollisionTestArm.robot()
      # Configuration that's close but not quite colliding
      positions = %{shoulder: 0.0, elbow: 2.5}

      # Without margin, no collision
      refute Collision.self_collision?(robot, positions)

      # With large margin, should detect near-collision
      assert Collision.self_collision?(robot, positions, margin: 0.1)
    end
  end

  describe "detect_self_collisions/3" do
    test "returns empty list when no collisions" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      assert [] = Collision.detect_self_collisions(robot, positions)
    end

    test "returns collision info when colliding" do
      robot = CollisionTestArm.robot()
      # Use large margin to force collision
      positions = %{shoulder: 0.0, elbow: 0.0}

      collisions = Collision.detect_self_collisions(robot, positions, margin: 0.5)

      assert [collision | _] = collisions
      assert is_atom(collision.link_a)
      assert is_atom(collision.link_b)
      assert is_float(collision.penetration_depth)
      assert collision.penetration_depth > 0
    end

    test "does not report collisions between adjacent links" do
      robot = CollisionTestArm.robot()
      # Even with extreme angles, adjacent links should not report collision
      positions = %{shoulder: 0.0, elbow: 0.5}

      collisions = Collision.detect_self_collisions(robot, positions)

      for collision <- collisions do
        # No collision should be between adjacent pairs
        refute {collision.link_a, collision.link_b} in [
                 {:base, :upper_arm},
                 {:upper_arm, :base},
                 {:upper_arm, :forearm},
                 {:forearm, :upper_arm}
               ]
      end
    end
  end

  describe "obstacle/3 and obstacle/4" do
    test "creates sphere obstacle" do
      centre = Vec3.new(1.0, 2.0, 3.0)
      obstacle = Collision.obstacle(:sphere, centre, 0.5)

      assert obstacle.type == :sphere
      assert {:sphere, ^centre, 0.5} = obstacle.geometry
      assert {min_pt, max_pt} = obstacle.aabb
      assert_in_delta Vec3.x(min_pt), 0.5, 0.001
      assert_in_delta Vec3.x(max_pt), 1.5, 0.001
    end

    test "creates capsule obstacle" do
      point_a = Vec3.new(0.0, 0.0, 0.0)
      point_b = Vec3.new(1.0, 0.0, 0.0)
      obstacle = Collision.obstacle(:capsule, point_a, point_b, 0.1)

      assert obstacle.type == :capsule
      assert {:capsule, ^point_a, ^point_b, 0.1} = obstacle.geometry
    end

    test "creates axis-aligned box obstacle" do
      centre = Vec3.new(0.0, 0.0, 0.0)
      half_extents = Vec3.new(1.0, 0.5, 0.25)
      obstacle = Collision.obstacle(:box, centre, half_extents)

      assert obstacle.type == :box
      {min_pt, max_pt} = obstacle.aabb
      assert_in_delta Vec3.x(min_pt), -1.0, 0.001
      assert_in_delta Vec3.y(min_pt), -0.5, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 0.5, 0.001
    end

    test "creates oriented box obstacle" do
      centre = Vec3.new(0.0, 0.0, 0.0)
      half_extents = Vec3.new(1.0, 0.5, 0.25)
      rotation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 4)
      obstacle = Collision.obstacle(:box, centre, half_extents, rotation)

      assert obstacle.type == :box
      # Rotated box AABB should be larger than axis-aligned version
      {min_pt, max_pt} = obstacle.aabb
      # When rotated 45 degrees, the 1.0 and 0.5 extents project to about 1.06 in X and Y
      assert Vec3.x(min_pt) < -1.0
      assert Vec3.x(max_pt) > 1.0
    end
  end

  describe "collides_with?/4" do
    test "returns false when robot doesn't collide with obstacles" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      # Obstacle far from robot
      obstacles = [Collision.obstacle(:sphere, Vec3.new(10.0, 0.0, 0.0), 0.1)]

      refute Collision.collides_with?(robot, positions, obstacles)
    end

    test "returns true when robot collides with obstacle" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      # Obstacle at robot base position
      obstacles = [Collision.obstacle(:sphere, Vec3.new(0.0, 0.0, 0.0), 0.2)]

      assert Collision.collides_with?(robot, positions, obstacles)
    end

    test "returns false with empty obstacle list" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      refute Collision.collides_with?(robot, positions, [])
    end
  end

  describe "detect_collisions/4" do
    test "returns empty list when no collisions" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      obstacles = [Collision.obstacle(:sphere, Vec3.new(10.0, 0.0, 0.0), 0.1)]

      assert [] = Collision.detect_collisions(robot, positions, obstacles)
    end

    test "returns collision info when robot collides with obstacle" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      # Obstacle overlapping with base
      obstacles = [Collision.obstacle(:sphere, Vec3.new(0.0, 0.0, 0.0), 0.2)]

      collisions = Collision.detect_collisions(robot, positions, obstacles)

      assert [collision | _] = collisions
      assert collision.link_b == :environment
      assert is_float(collision.penetration_depth)
    end

    test "detects collision with multiple obstacles" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      obstacles = [
        Collision.obstacle(:sphere, Vec3.new(0.0, 0.0, 0.0), 0.2),
        Collision.obstacle(:sphere, Vec3.new(0.3, 0.0, 0.1), 0.1)
      ]

      collisions = Collision.detect_collisions(robot, positions, obstacles)

      # Should detect collisions with both obstacles
      assert length(collisions) >= 2
    end

    test "respects margin parameter" do
      robot = CollisionTestArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0}

      # Obstacle below the robot (base is at z=0, arm extends in +X/+Z direction)
      # Place sphere at z=-0.1, just below the base (base extends to z=-0.05)
      obstacles = [Collision.obstacle(:sphere, Vec3.new(0.0, 0.0, -0.08), 0.01)]

      # Without margin, no collision (0.02 gap from base)
      assert [] = Collision.detect_collisions(robot, positions, obstacles)

      # With margin, should detect near-collision
      collisions = Collision.detect_collisions(robot, positions, obstacles, margin: 0.03)
      assert [_ | _] = collisions
    end
  end
end
