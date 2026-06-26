# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.KinematicsTest do
  use ExUnit.Case, async: true
  import BB.Unit

  alias BB.ExampleRobots.SixDofArm
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot.{Kinematics, State}

  defmodule PlanarArm do
    @moduledoc """
    A 2-DOF planar arm in the XZ plane.

    - Base at origin
    - Joint 1 rotates around Y axis at (0, 0, 0)
    - Link 1 extends 1m along X axis
    - Joint 2 rotates around Y axis at (1, 0, 0) in link1 frame
    - Link 2 extends 1m along X axis
    - End effector at (1, 0, 0) in link2 frame

    Using standard rotation convention (ROS/URDF):
    - +90° around Y takes X → -Z (counterclockwise when viewed from +Y)

    With all joints at 0: end effector at (2, 0, 0)
    With joint1 at 90°: end effector at (0, 0, -2)
    With joint1 at -90°: end effector at (0, 0, 2)
    With joint1 at 90°, joint2 at -90°: end effector at (1, 0, -1)
    """
    use BB

    topology do
      link :base do
        joint :joint1 do
          type :revolute

          axis do
            roll(~u(-90 degree))
          end

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(1 degree_per_second))
          end

          link :link1 do
            joint :joint2 do
              type :revolute

              origin do
                x(~u(1 meter))
              end

              axis do
                roll(~u(-90 degree))
              end

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(1 degree_per_second))
              end

              link :link2 do
                joint :end_joint do
                  type :fixed

                  origin do
                    x(~u(1 meter))
                  end

                  link :end_effector
                end
              end
            end
          end
        end
      end
    end
  end

  defmodule VerticalArm do
    @moduledoc """
    A 3-DOF vertical arm that rotates around Z and extends along Z.

    - Base at origin
    - Joint 1 at z=0.1m, rotates around Z
    - Link 1
    - Joint 2 at z=0.5m from link1, rotates around Z
    - Link 2
    - Joint 3 at z=0.4m from link2, rotates around Z
    - End effector

    All joints at 0: end effector at (0, 0, 1.0)
    Since all rotations are around Z and all offsets are along Z,
    rotating joints only affects orientation, not position.
    """
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          origin do
            z(~u(10 centimeter))
          end

          axis do
          end

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(1 degree_per_second))
          end

          link :upper_arm do
            joint :elbow do
              type :revolute

              origin do
                z(~u(50 centimeter))
              end

              axis do
              end

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(1 degree_per_second))
              end

              link :forearm do
                joint :wrist do
                  type :revolute

                  origin do
                    z(~u(40 centimeter))
                  end

                  axis do
                  end

                  limit do
                    effort(~u(10 newton_meter))
                    velocity(~u(1 degree_per_second))
                  end

                  link :end_effector
                end
              end
            end
          end
        end
      end
    end
  end

  defmodule PrismaticSlider do
    @moduledoc """
    A robot with a prismatic joint for testing linear motion.

    - Base at origin
    - Prismatic joint slides along Z axis
    - End effector
    """
    use BB

    topology do
      link :base do
        joint :slider do
          type :prismatic

          axis do
          end

          limit do
            lower(~u(0 meter))
            upper(~u(1 meter))
            effort(~u(10 newton))
            velocity(~u(1 meter_per_second))
          end

          link :platform
        end
      end
    end
  end

  describe "planar arm forward kinematics" do
    test "all joints at zero - arm extends along X" do
      robot = PlanarArm.robot()
      positions = %{joint1: 0.0, joint2: 0.0, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      assert_in_delta x, 2.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 0.0, 0.0001
    end

    test "joint1 at 90 degrees - arm points along -Z" do
      robot = PlanarArm.robot()
      positions = %{joint1: :math.pi() / 2, joint2: 0.0, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # +90° around Y takes X → -Z
      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, -2.0, 0.0001
    end

    test "joint1 at -90 degrees - arm points along +Z" do
      robot = PlanarArm.robot()
      positions = %{joint1: -:math.pi() / 2, joint2: 0.0, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # -90° around Y takes X → +Z
      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 2.0, 0.0001
    end

    test "joint1 at 180 degrees - arm points along -X" do
      robot = PlanarArm.robot()
      positions = %{joint1: :math.pi(), joint2: 0.0, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      assert_in_delta x, -2.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 0.0, 0.0001
    end

    test "joint2 at 90 degrees - second link bends down" do
      robot = PlanarArm.robot()
      positions = %{joint1: 0.0, joint2: :math.pi() / 2, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # +90° around Y takes local X → -Z
      assert_in_delta x, 1.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, -1.0, 0.0001
    end

    test "joint2 at -90 degrees - second link bends up" do
      robot = PlanarArm.robot()
      positions = %{joint1: 0.0, joint2: -:math.pi() / 2, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # -90° around Y takes local X → +Z
      assert_in_delta x, 1.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 1.0, 0.0001
    end

    test "joint1 at 90, joint2 at -90 - arm folds back" do
      robot = PlanarArm.robot()
      positions = %{joint1: :math.pi() / 2, joint2: -:math.pi() / 2, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # joint1 +90° takes arm along -Z, joint2 -90° rotates link2 back toward +X
      assert_in_delta x, 1.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, -1.0, 0.0001
    end

    test "joint1 at 45, joint2 at 45 - diagonal configuration" do
      robot = PlanarArm.robot()
      angle = :math.pi() / 4
      positions = %{joint1: angle, joint2: angle, end_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # Standard convention: +θ around Y takes X toward -Z
      # After joint1: link1 extends in direction (cos θ, 0, -sin θ)
      # After both joints (total 2θ): link2 extends in direction (cos 2θ, 0, -sin 2θ)
      link1_end_x = 1.0 * :math.cos(angle)
      link1_end_z = -1.0 * :math.sin(angle)

      link2_end_x = link1_end_x + 1.0 * :math.cos(2 * angle)
      link2_end_z = link1_end_z + -1.0 * :math.sin(2 * angle)

      assert_in_delta x, link2_end_x, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, link2_end_z, 0.0001
    end

    test "intermediate links have correct positions" do
      robot = PlanarArm.robot()
      positions = %{joint1: :math.pi() / 2, joint2: 0.0, end_joint: 0.0}

      # link1's frame is at joint1's origin (0, 0, 0)
      {x1, y1, z1} = Kinematics.link_position(robot, positions, :link1)
      assert_in_delta x1, 0.0, 0.0001
      assert_in_delta y1, 0.0, 0.0001
      assert_in_delta z1, 0.0, 0.0001

      # link2's frame is at joint2's origin, which is (1,0,0) in link1 frame
      # After joint1 +90° rotation: (1,0,0) in rotated frame becomes (0,0,-1) in base
      {x2, y2, z2} = Kinematics.link_position(robot, positions, :link2)
      assert_in_delta x2, 0.0, 0.0001
      assert_in_delta y2, 0.0, 0.0001
      assert_in_delta z2, -1.0, 0.0001
    end
  end

  describe "vertical arm forward kinematics" do
    test "all joints at zero - arm extends along Z" do
      robot = VerticalArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0, wrist: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 1.0, 0.0001
    end

    test "rotating around Z doesn't change position when offset is along Z" do
      robot = VerticalArm.robot()

      for angle <- [0.0, :math.pi() / 4, :math.pi() / 2, :math.pi()] do
        positions = %{shoulder: angle, elbow: 0.0, wrist: 0.0}
        {x, y, z} = Kinematics.link_position(robot, positions, :end_effector)

        assert_in_delta x, 0.0, 0.0001, "x should be 0 for shoulder angle #{angle}"
        assert_in_delta y, 0.0, 0.0001, "y should be 0 for shoulder angle #{angle}"
        assert_in_delta z, 1.0, 0.0001, "z should be 1.0 for shoulder angle #{angle}"
      end
    end

    test "intermediate link positions are correct" do
      robot = VerticalArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0, wrist: 0.0}

      {_, _, z_upper} = Kinematics.link_position(robot, positions, :upper_arm)
      assert_in_delta z_upper, 0.1, 0.0001

      {_, _, z_forearm} = Kinematics.link_position(robot, positions, :forearm)
      assert_in_delta z_forearm, 0.6, 0.0001
    end
  end

  describe "prismatic joint forward kinematics" do
    test "zero position - platform at origin" do
      robot = PrismaticSlider.robot()
      positions = %{slider: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :platform)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 0.0, 0.0001
    end

    test "positive displacement - platform moves along Z" do
      robot = PrismaticSlider.robot()
      positions = %{slider: 0.5}

      {x, y, z} = Kinematics.link_position(robot, positions, :platform)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 0.5, 0.0001
    end

    test "full extension" do
      robot = PrismaticSlider.robot()
      positions = %{slider: 1.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :platform)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 1.0, 0.0001
    end
  end

  describe "all_link_transforms consistency" do
    test "all_link_transforms matches individual forward_kinematics" do
      robot = PlanarArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :joint1, :math.pi() / 4)
      State.set_joint_position(state, :joint2, :math.pi() / 6)

      all_transforms = Kinematics.all_link_transforms(robot, state)

      for link_name <- [:base, :link1, :link2, :end_effector] do
        fk_transform = Kinematics.forward_kinematics(robot, state, link_name)
        all_transform = all_transforms[link_name]

        fk_pos = Transform.get_translation(fk_transform)
        all_pos = Transform.get_translation(all_transform)

        assert_in_delta Vec3.x(fk_pos), Vec3.x(all_pos), 0.0001, "x mismatch for #{link_name}"
        assert_in_delta Vec3.y(fk_pos), Vec3.y(all_pos), 0.0001, "y mismatch for #{link_name}"
        assert_in_delta Vec3.z(fk_pos), Vec3.z(all_pos), 0.0001, "z mismatch for #{link_name}"
      end

      State.delete(state)
    end
  end

  describe "transform composition" do
    test "forward then inverse returns to origin" do
      robot = PlanarArm.robot()
      positions = %{joint1: :math.pi() / 3, joint2: :math.pi() / 4, end_joint: 0.0}

      transform = Kinematics.forward_kinematics(robot, positions, :end_effector)
      inverse = Transform.inverse(transform)
      composed = Transform.compose(transform, inverse)

      identity_tensor = Transform.tensor(Transform.identity())
      composed_tensor = Transform.tensor(composed)

      for i <- 0..3, j <- 0..3 do
        expected = Nx.to_number(identity_tensor[i][j])
        actual = Nx.to_number(composed_tensor[i][j])
        assert_in_delta actual, expected, 0.0001, "mismatch at [#{i}][#{j}]"
      end
    end
  end

  describe "edge cases" do
    test "base link is always at identity" do
      robot = PlanarArm.robot()

      for angle <- [0.0, :math.pi() / 2, :math.pi()] do
        positions = %{joint1: angle, joint2: angle, end_joint: 0.0}
        {x, y, z} = Kinematics.link_position(robot, positions, :base)

        assert_in_delta x, 0.0, 0.0001
        assert_in_delta y, 0.0, 0.0001
        assert_in_delta z, 0.0, 0.0001
      end
    end

    test "missing joint position defaults to zero" do
      robot = PlanarArm.robot()
      # Only provide joint1, others default to 0
      positions = %{joint1: :math.pi() / 2}

      {x, _y, z} = Kinematics.link_position(robot, positions, :end_effector)

      # With joint1 at +90° and others at 0, arm extends along -Z
      assert_in_delta x, 0.0, 0.0001
      assert_in_delta z, -2.0, 0.0001
    end
  end

  describe "position_jacobian/4" do
    @sixdof_joints [
      :shoulder_pan_joint,
      :shoulder_lift_joint,
      :elbow_joint,
      :wrist_1_joint,
      :wrist_2_joint,
      :wrist_3_joint
    ]
    @sixdof_positions %{
      shoulder_pan_joint: 0.3,
      shoulder_lift_joint: -0.5,
      elbow_joint: 0.8,
      wrist_1_joint: -0.4,
      wrist_2_joint: 0.6,
      wrist_3_joint: 0.2
    }

    test "matches central finite differences" do
      robot = SixDofArm.robot()

      analytical = Kinematics.position_jacobian(robot, @sixdof_positions, :tool0, @sixdof_joints)
      finite_diff = finite_difference_jacobian(robot, @sixdof_positions, :tool0, @sixdof_joints)

      for column <- 0..(length(@sixdof_joints) - 1), row <- 0..2 do
        assert_in_delta Nx.to_number(analytical[row][column]),
                        Nx.to_number(finite_diff[row][column]),
                        1.0e-6,
                        "mismatch at [#{row}][#{column}]"
      end
    end

    test "follows joint_names order and zeroes off-chain joints" do
      robot = SixDofArm.robot()

      reordered = [:elbow_joint, :shoulder_pan_joint]
      base = Kinematics.position_jacobian(robot, @sixdof_positions, :tool0, @sixdof_joints)
      picked = Kinematics.position_jacobian(robot, @sixdof_positions, :tool0, reordered)

      assert Nx.to_flat_list(picked[[.., 0]]) == Nx.to_flat_list(base[[.., 2]])
      assert Nx.to_flat_list(picked[[.., 1]]) == Nx.to_flat_list(base[[.., 0]])

      # A joint that does not lie on the chain to the target gets a zero column.
      with_unrelated =
        Kinematics.position_jacobian(robot, @sixdof_positions, :tool0, [
          :wrist_3_joint,
          :not_a_real_joint
        ])

      assert Nx.to_flat_list(with_unrelated[[.., 1]]) == [0.0, 0.0, 0.0]
    end
  end

  describe "defn chain matches eager joint composition" do
    # `forward_kinematics/3` runs the chain through `BB.Robot.Kinematics.Defn`.
    # This pins it against the eager per-joint composition it replaced, so the
    # two cannot silently diverge.
    cases = [
      {BB.ExampleRobots.SixDofArm, :tool0,
       %{
         shoulder_pan_joint: 0.3,
         shoulder_lift_joint: -0.5,
         elbow_joint: 0.8,
         wrist_1_joint: -0.4,
         wrist_2_joint: 0.6,
         wrist_3_joint: 0.2
       }},
      {BB.ExampleRobots.PanTiltCamera, :camera_link, %{pan_joint: 0.4, tilt_joint: -0.6}},
      {BB.ExampleRobots.LinearActuator, :slider_link, %{slider_joint: 0.15}},
      {BB.ExampleRobots.CollisionTestArm, :forearm, %{shoulder: 0.7, elbow: -0.9}}
    ]

    for {module, target, positions} <- cases do
      test "#{inspect(module)} -> #{target}" do
        robot = unquote(module).robot()
        positions = unquote(Macro.escape(positions))

        actual =
          Transform.tensor(Kinematics.forward_kinematics(robot, positions, unquote(target)))

        expected = Transform.tensor(reference_chain(robot, positions, unquote(target)))

        for i <- 0..3, j <- 0..3 do
          assert_in_delta Nx.to_number(actual[i][j]),
                          Nx.to_number(expected[i][j]),
                          1.0e-9,
                          "mismatch at [#{i}][#{j}]"
        end
      end
    end
  end

  describe "all_link_transforms defn matches eager joint composition" do
    # `all_link_transforms/2` runs the whole tree through
    # `BB.Robot.Kinematics.Defn.link_transforms/7`. This pins it against the
    # eager parent-walk it replaced, including a branching robot.
    cases = [
      {BB.ExampleRobots.SixDofArm,
       %{
         shoulder_pan_joint: 0.3,
         shoulder_lift_joint: -0.5,
         elbow_joint: 0.8,
         wrist_1_joint: -0.4,
         wrist_2_joint: 0.6,
         wrist_3_joint: 0.2
       }},
      {BB.ExampleRobots.DifferentialDriveRobot, %{left_wheel_joint: 1.2, right_wheel_joint: -0.7}}
    ]

    for {module, positions} <- cases do
      test "#{inspect(module)}" do
        robot = unquote(module).robot()
        positions = unquote(Macro.escape(positions))

        actual = Kinematics.all_link_transforms(robot, positions)
        expected = reference_all_links(robot, positions)

        for link_name <- robot.topology.link_order do
          actual_tensor = Transform.tensor(Map.fetch!(actual, link_name))
          expected_tensor = Transform.tensor(Map.fetch!(expected, link_name))

          for i <- 0..3, j <- 0..3 do
            assert_in_delta Nx.to_number(actual_tensor[i][j]),
                            Nx.to_number(expected_tensor[i][j]),
                            1.0e-9,
                            "#{link_name} mismatch at [#{i}][#{j}]"
          end
        end
      end
    end
  end

  defp finite_difference_jacobian(robot, positions, target_link, joint_names) do
    epsilon = 1.0e-6

    joint_names
    |> Enum.map(fn joint_name ->
      current = Map.get(positions, joint_name, 0.0)

      {xp, yp, zp} =
        Kinematics.link_position(
          robot,
          Map.put(positions, joint_name, current + epsilon),
          target_link
        )

      {xm, ym, zm} =
        Kinematics.link_position(
          robot,
          Map.put(positions, joint_name, current - epsilon),
          target_link
        )

      [(xp - xm) / (2 * epsilon), (yp - ym) / (2 * epsilon), (zp - zm) / (2 * epsilon)]
    end)
    |> Nx.tensor(type: :f64)
    |> Nx.transpose()
  end

  defp reference_chain(robot, positions, target_link) do
    robot
    |> BB.Robot.path_to(target_link)
    |> Enum.filter(&Map.has_key?(robot.joints, &1))
    |> Enum.reduce(Transform.identity(), fn joint_name, acc ->
      Transform.compose(acc, Kinematics.compute_joint_transform(robot, positions, joint_name))
    end)
  end

  defp reference_all_links(robot, positions) do
    Enum.reduce(robot.topology.link_order, %{}, fn link_name, transforms ->
      transform =
        case BB.Robot.get_link(robot, link_name) do
          %{parent_joint: nil} ->
            Transform.identity()

          %{parent_joint: parent_joint_name} ->
            parent_link = robot.joints[parent_joint_name].parent_link

            Transform.compose(
              Map.fetch!(transforms, parent_link),
              Kinematics.compute_joint_transform(robot, positions, parent_joint_name)
            )
        end

      Map.put(transforms, link_name, transform)
    end)
  end
end
