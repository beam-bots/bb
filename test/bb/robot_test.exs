# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.RobotTest do
  use ExUnit.Case, async: true
  import BB.Unit

  alias BB.Error.Kinematics.NoParentJoint
  alias BB.Error.Kinematics.NotAnAncestor
  alias BB.Error.Kinematics.UnknownJoint
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.{Joint, Kinematics, Link, State, Topology}

  defmodule SimpleArm do
    use BB

    topology do
      link :base do
        inertial do
          mass(~u(5 kilogram))
        end

        joint :shoulder do
          type :revolute

          origin do
            z(~u(10 centimeter))
          end

          axis do
          end

          limit do
            lower(~u(-90 degree))
            upper(~u(90 degree))
            effort(~u(50 newton_meter))
            velocity(~u(2 degree_per_second))
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
                lower(~u(0 degree))
                upper(~u(135 degree))
                effort(~u(30 newton_meter))
                velocity(~u(3 degree_per_second))
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
                    velocity(~u(5 degree_per_second))
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

  describe "Robot struct" do
    test "robot/0 returns the optimised robot" do
      robot = SimpleArm.robot()
      assert %Robot{} = robot
      assert robot.name == SimpleArm
      assert robot.root_link == :base
    end

    test "contains all links" do
      robot = SimpleArm.robot()
      assert Map.has_key?(robot.links, :base)
      assert Map.has_key?(robot.links, :upper_arm)
      assert Map.has_key?(robot.links, :forearm)
      assert Map.has_key?(robot.links, :end_effector)
      assert map_size(robot.links) == 4
    end

    test "contains all joints" do
      robot = SimpleArm.robot()
      assert Map.has_key?(robot.joints, :shoulder)
      assert Map.has_key?(robot.joints, :elbow)
      assert Map.has_key?(robot.joints, :wrist)
      assert map_size(robot.joints) == 3
    end

    test "root_link/1 returns the root of the kinematic tree" do
      assert Robot.root_link(SimpleArm.robot()) == :base
    end

    test "get_link/2 returns correct link" do
      robot = SimpleArm.robot()
      assert {:ok, %Link{name: :upper_arm}} = Robot.get_link(robot, :upper_arm)
    end

    test "get_link/2 names the robot when the link doesn't exist" do
      robot = SimpleArm.robot()

      assert {:error, %UnknownLink{link: :nope, robot: robot_name}} =
               Robot.get_link(robot, :nope)

      assert robot_name == robot.name
    end

    test "get_joint/2 returns correct joint" do
      robot = SimpleArm.robot()
      assert {:ok, %Joint{name: :elbow, type: :revolute}} = Robot.get_joint(robot, :elbow)
    end

    test "get_joint/2 reports an unknown joint as a joint, not a link" do
      assert {:error, %UnknownJoint{joint: :nope}} = Robot.get_joint(SimpleArm.robot(), :nope)
    end

    test "parent_joint/2 returns parent joint of a link" do
      robot = SimpleArm.robot()
      assert {:ok, %Joint{name: :shoulder}} = Robot.parent_joint(robot, :upper_arm)
      assert {:ok, %Joint{name: :elbow}} = Robot.parent_joint(robot, :forearm)
    end

    # The root having no parent is a structural fact, so a caller walking up the
    # tree gets a termination signal rather than "that link doesn't exist".
    test "parent_joint/2 distinguishes the root from an unknown link" do
      robot = SimpleArm.robot()

      assert {:error, %NoParentJoint{link: :base}} = Robot.parent_joint(robot, :base)
      assert {:error, %UnknownLink{link: :nope}} = Robot.parent_joint(robot, :nope)
    end

    test "child_joints/2 returns child joints of a link" do
      robot = SimpleArm.robot()
      assert {:ok, [%Joint{name: :shoulder}]} = Robot.child_joints(robot, :base)
    end

    test "child_joints/2 distinguishes no children from no such link" do
      robot = SimpleArm.robot()

      assert {:ok, []} = Robot.child_joints(robot, :end_effector)
      assert {:error, %UnknownLink{link: :nope}} = Robot.child_joints(robot, :nope)
    end

    test "path_to/2 returns path from root" do
      robot = SimpleArm.robot()
      assert Robot.path_to(robot, :base) == {:ok, [:base]}
      assert Robot.path_to(robot, :shoulder) == {:ok, [:base, :shoulder]}
      assert Robot.path_to(robot, :upper_arm) == {:ok, [:base, :shoulder, :upper_arm]}

      assert Robot.path_to(robot, :end_effector) ==
               {:ok,
                [
                  :base,
                  :shoulder,
                  :upper_arm,
                  :elbow,
                  :forearm,
                  :wrist,
                  :end_effector
                ]}
    end

    test "path_to/2 errors on an unknown link" do
      assert {:error, %UnknownLink{link: :nope}} = Robot.path_to(SimpleArm.robot(), :nope)
    end

    test "path_between/3 drops the prefix above the source" do
      robot = SimpleArm.robot()

      assert Robot.path_between(robot, :upper_arm, :end_effector) ==
               {:ok, [:upper_arm, :elbow, :forearm, :wrist, :end_effector]}
    end

    test "path_between/3 from the root agrees with path_to/2" do
      robot = SimpleArm.robot()

      assert Robot.path_between(robot, Robot.root_link(robot), :end_effector) ==
               Robot.path_to(robot, :end_effector)
    end

    test "path_between/3 from a link to itself is that link alone" do
      assert Robot.path_between(SimpleArm.robot(), :forearm, :forearm) == {:ok, [:forearm]}
    end

    # Reversing the ends is the easy mistake, and the error should say which
    # link the caller should have passed rather than just reporting a failure.
    test "path_between/3 reports the nearest common ancestor when the source is below the target" do
      robot = SimpleArm.robot()

      assert {:error,
              %NotAnAncestor{
                source_link: :end_effector,
                target_link: :upper_arm,
                common_ancestor: :upper_arm
              }} = Robot.path_between(robot, :end_effector, :upper_arm)
    end

    test "path_between/3 attributes an unknown link to the right end" do
      robot = SimpleArm.robot()

      assert {:error, %UnknownLink{link: :nope, role: :source}} =
               Robot.path_between(robot, :nope, :forearm)

      assert {:error, %UnknownLink{link: :nope, role: :target}} =
               Robot.path_between(robot, :forearm, :nope)
    end

    # A single chain can't produce a branch point, so the sibling case needs a
    # topology that actually branches.
    test "path_between/3 names the branch point for two siblings" do
      robot = BB.ExampleRobots.DifferentialDriveRobot.robot()

      assert {:error,
              %NotAnAncestor{
                source_link: :left_wheel,
                target_link: :right_wheel,
                common_ancestor: :base_link
              }} = Robot.path_between(robot, :left_wheel, :right_wheel)
    end

    test "path_between/3 descends into one branch without picking up the others" do
      robot = BB.ExampleRobots.DifferentialDriveRobot.robot()

      assert Robot.path_between(robot, :base_link, :right_wheel) ==
               {:ok, [:base_link, :right_wheel_joint, :right_wheel]}
    end

    test "links_in_order/1 returns links in topological order" do
      robot = SimpleArm.robot()
      links = Robot.links_in_order(robot)
      link_names = Enum.map(links, & &1.name)
      assert link_names == [:base, :upper_arm, :forearm, :end_effector]
    end

    test "joints_in_order/1 returns joints in traversal order" do
      robot = SimpleArm.robot()
      joints = Robot.joints_in_order(robot)
      joint_names = Enum.map(joints, & &1.name)
      assert joint_names == [:shoulder, :elbow, :wrist]
    end
  end

  describe "Link struct" do
    test "has parent and child references" do
      robot = SimpleArm.robot()
      {:ok, upper_arm} = Robot.get_link(robot, :upper_arm)

      assert upper_arm.parent_joint == :shoulder
      assert upper_arm.child_joints == [:elbow]
    end

    test "root link has no parent" do
      robot = SimpleArm.robot()
      {:ok, base} = Robot.get_link(robot, :base)

      assert base.parent_joint == nil
      assert base.child_joints == [:shoulder]
    end

    test "leaf link has no children" do
      robot = SimpleArm.robot()
      {:ok, end_effector} = Robot.get_link(robot, :end_effector)

      assert end_effector.child_joints == []
    end

    test "mass is converted to kilograms" do
      robot = SimpleArm.robot()
      {:ok, base} = Robot.get_link(robot, :base)

      assert base.mass == 5.0
    end
  end

  describe "Joint struct" do
    test "has parent and child link references" do
      robot = SimpleArm.robot()
      {:ok, elbow} = Robot.get_joint(robot, :elbow)

      assert elbow.parent_link == :upper_arm
      assert elbow.child_link == :forearm
    end

    test "origin is converted to meters and radians" do
      robot = SimpleArm.robot()
      {:ok, shoulder} = Robot.get_joint(robot, :shoulder)

      assert shoulder.origin.position == {0.0, 0.0, 0.1}
      assert shoulder.origin.orientation == {0.0, 0.0, 0.0}
    end

    test "axis is normalised" do
      robot = SimpleArm.robot()
      {:ok, shoulder} = Robot.get_joint(robot, :shoulder)

      assert shoulder.axis == {0.0, 0.0, 1.0}
    end

    test "limits are converted to radians" do
      robot = SimpleArm.robot()
      {:ok, shoulder} = Robot.get_joint(robot, :shoulder)

      assert_in_delta shoulder.limits.lower, -:math.pi() / 2, 0.001
      assert_in_delta shoulder.limits.upper, :math.pi() / 2, 0.001
    end

    test "velocity is converted to radians per second" do
      robot = SimpleArm.robot()
      {:ok, shoulder} = Robot.get_joint(robot, :shoulder)

      expected = 2.0 * :math.pi() / 180.0
      assert_in_delta shoulder.limits.velocity, expected, 0.0001
    end

    test "rotational?/1 returns true for revolute joints" do
      robot = SimpleArm.robot()
      {:ok, shoulder} = Robot.get_joint(robot, :shoulder)
      assert Joint.rotational?(shoulder)
    end

    test "movable?/1 returns true for non-fixed joints" do
      robot = SimpleArm.robot()
      {:ok, shoulder} = Robot.get_joint(robot, :shoulder)
      assert Joint.movable?(shoulder)
    end
  end

  describe "Topology struct" do
    test "depth_of/2 returns correct depth" do
      robot = SimpleArm.robot()
      topology = robot.topology

      assert Topology.depth_of(topology, :base) == {:ok, 0}
      assert Topology.depth_of(topology, :shoulder) == {:ok, 1}
      assert Topology.depth_of(topology, :upper_arm) == {:ok, 1}
      assert Topology.depth_of(topology, :elbow) == {:ok, 2}
    end

    test "depth_of/2 errors on an unknown node" do
      topology = SimpleArm.robot().topology

      assert {:error, %UnknownLink{link: :nope}} = Topology.depth_of(topology, :nope)
    end

    test "max_depth/1 returns maximum depth" do
      robot = SimpleArm.robot()
      assert Topology.max_depth(robot.topology) == 3
    end

    test "leaf_links/2 returns links with no children" do
      robot = SimpleArm.robot()
      leaves = Topology.leaf_links(robot.topology, robot)

      assert leaves == [:end_effector]
    end
  end

  describe "Transform" do
    test "identity/0 returns 4x4 identity matrix" do
      t = Transform.identity()
      assert Nx.shape(Transform.tensor(t)) == {4, 4}

      expected = Nx.eye(4, type: :f64)
      assert Nx.to_list(Transform.tensor(t)) == Nx.to_list(expected)
    end

    test "translation/1 creates translation matrix" do
      t = Transform.translation(Vec3.new(1.0, 2.0, 3.0))
      pos = Transform.get_translation(t)
      assert Vec3.x(pos) == 1.0
      assert Vec3.y(pos) == 2.0
      assert Vec3.z(pos) == 3.0
    end

    test "rotation_x/1 rotates around X axis" do
      t = Transform.rotation_x(:math.pi() / 2)
      result = Transform.apply_to_point(t, Vec3.new(0.0, 1.0, 0.0))

      assert_in_delta Vec3.x(result), 0.0, 0.0001
      assert_in_delta Vec3.y(result), 0.0, 0.0001
      assert_in_delta Vec3.z(result), 1.0, 0.0001
    end

    test "rotation_y/1 rotates around Y axis" do
      t = Transform.rotation_y(:math.pi() / 2)
      result = Transform.apply_to_point(t, Vec3.new(1.0, 0.0, 0.0))

      assert_in_delta Vec3.x(result), 0.0, 0.0001
      assert_in_delta Vec3.y(result), 0.0, 0.0001
      assert_in_delta Vec3.z(result), -1.0, 0.0001
    end

    test "rotation_z/1 rotates around Z axis" do
      t = Transform.rotation_z(:math.pi() / 2)
      result = Transform.apply_to_point(t, Vec3.new(1.0, 0.0, 0.0))

      assert_in_delta Vec3.x(result), 0.0, 0.0001
      assert_in_delta Vec3.y(result), 1.0, 0.0001
      assert_in_delta Vec3.z(result), 0.0, 0.0001
    end

    test "compose/2 multiplies transforms" do
      t1 = Transform.translation(Vec3.new(1.0, 0.0, 0.0))
      t2 = Transform.translation(Vec3.new(0.0, 2.0, 0.0))
      t = Transform.compose(t1, t2)

      pos = Transform.get_translation(t)
      assert Vec3.x(pos) == 1.0
      assert Vec3.y(pos) == 2.0
      assert Vec3.z(pos) == 0.0
    end

    test "compose_all/1 composes multiple transforms" do
      transforms = [
        Transform.translation(Vec3.new(1.0, 0.0, 0.0)),
        Transform.translation(Vec3.new(0.0, 1.0, 0.0)),
        Transform.translation(Vec3.new(0.0, 0.0, 1.0))
      ]

      t = Transform.compose_all(transforms)
      pos = Transform.get_translation(t)
      assert Vec3.x(pos) == 1.0
      assert Vec3.y(pos) == 1.0
      assert Vec3.z(pos) == 1.0
    end

    test "inverse/1 computes inverse transform" do
      t = Transform.translation(Vec3.new(1.0, 2.0, 3.0))
      t_inv = Transform.inverse(t)

      pos = Transform.get_translation(t_inv)
      assert_in_delta Vec3.x(pos), -1.0, 0.0001
      assert_in_delta Vec3.y(pos), -2.0, 0.0001
      assert_in_delta Vec3.z(pos), -3.0, 0.0001
    end

    test "from_axis_angle/2 rotates around arbitrary axis" do
      axis = Vec3.new(0.0, 0.0, 1.0)
      t = Transform.from_axis_angle(axis, :math.pi() / 2)
      result = Transform.apply_to_point(t, Vec3.new(1.0, 0.0, 0.0))

      assert_in_delta Vec3.x(result), 0.0, 0.0001
      assert_in_delta Vec3.y(result), 1.0, 0.0001
    end

    test "translation_along/2 translates along axis" do
      axis = Vec3.new(1.0, 0.0, 0.0)
      t = Transform.translation_along(axis, 2.5)

      pos = Transform.get_translation(t)
      assert Vec3.x(pos) == 2.5
      assert Vec3.y(pos) == 0.0
      assert Vec3.z(pos) == 0.0
    end

    test "from_origin/1 creates transform from position and orientation" do
      origin = %{
        position: {1.0, 2.0, 3.0},
        orientation: {0.0, 0.0, 0.0}
      }

      t = Transform.from_origin(origin)
      pos = Transform.get_translation(t)
      assert Vec3.x(pos) == 1.0
      assert Vec3.y(pos) == 2.0
      assert Vec3.z(pos) == 3.0
    end
  end

  describe "State" do
    test "new/1 creates state with zero positions" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      assert State.get_joint_position(state, :shoulder) == 0.0
      assert State.get_joint_position(state, :elbow) == 0.0
      assert State.get_joint_position(state, :wrist) == 0.0

      State.delete(state)
    end

    test "set_joint_position/3 and get_joint_position/2" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :shoulder, 0.5)
      assert State.get_joint_position(state, :shoulder) == 0.5

      State.delete(state)
    end

    test "set_joint_velocity/3 and get_joint_velocity/2" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_velocity(state, :shoulder, 1.5)
      assert State.get_joint_velocity(state, :shoulder) == 1.5

      State.delete(state)
    end

    test "get_all_positions/1 returns all joint positions" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :shoulder, 0.1)
      State.set_joint_position(state, :elbow, 0.2)
      State.set_joint_position(state, :wrist, 0.3)

      positions = State.get_all_positions(state)
      assert positions == %{shoulder: 0.1, elbow: 0.2, wrist: 0.3}

      State.delete(state)
    end

    test "set_positions/2 sets multiple positions at once" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_positions(state, %{shoulder: 0.5, elbow: 1.0})

      assert State.get_joint_position(state, :shoulder) == 0.5
      assert State.get_joint_position(state, :elbow) == 1.0

      State.delete(state)
    end

    test "reset/1 resets all positions to zero" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :shoulder, 0.5)
      State.reset(state)

      assert State.get_joint_position(state, :shoulder) == 0.0

      State.delete(state)
    end

    test "get_chain_positions/2 returns positions along path" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :shoulder, 0.1)
      State.set_joint_position(state, :elbow, 0.2)
      State.set_joint_position(state, :wrist, 0.3)

      positions = State.get_chain_positions(state, :end_effector)

      assert positions == [
               {:shoulder, 0.1},
               {:elbow, 0.2},
               {:wrist, 0.3}
             ]

      State.delete(state)
    end
  end

  describe "Kinematics" do
    test "forward_kinematics/3 with zero positions" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      transform = Kinematics.forward_kinematics(robot, state, :end_effector)
      pos = Transform.get_translation(transform)

      assert_in_delta Vec3.x(pos), 0.0, 0.0001
      assert_in_delta Vec3.y(pos), 0.0, 0.0001
      assert_in_delta Vec3.z(pos), 1.0, 0.0001

      State.delete(state)
    end

    test "forward_kinematics/3 with joint position" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :shoulder, :math.pi() / 2)

      transform = Kinematics.forward_kinematics(robot, state, :upper_arm)
      pos = Transform.get_translation(transform)

      assert_in_delta Vec3.x(pos), 0.0, 0.0001
      assert_in_delta Vec3.y(pos), 0.0, 0.0001
      assert_in_delta Vec3.z(pos), 0.1, 0.0001

      State.delete(state)
    end

    test "forward_kinematics/3 accepts position map" do
      robot = SimpleArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0, wrist: 0.0}

      transform = Kinematics.forward_kinematics(robot, positions, :end_effector)
      pos = Transform.get_translation(transform)

      assert_in_delta Vec3.z(pos), 1.0, 0.0001
    end

    test "all_link_transforms/2 returns transforms for all links" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      transforms = Kinematics.all_link_transforms(robot, state)

      assert Map.has_key?(transforms, :base)
      assert Map.has_key?(transforms, :upper_arm)
      assert Map.has_key?(transforms, :forearm)
      assert Map.has_key?(transforms, :end_effector)

      base_transform = transforms[:base]
      pos = Transform.get_translation(base_transform)
      assert {Vec3.x(pos), Vec3.y(pos), Vec3.z(pos)} == {0.0, 0.0, 0.0}

      State.delete(state)
    end

    test "link_position/3 returns link position" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      {x, y, z} = Kinematics.link_position(robot, state, :upper_arm)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 0.1, 0.0001

      State.delete(state)
    end

    test "raises for unknown link" do
      robot = SimpleArm.robot()

      assert_raise UnknownLink, ~r/Unknown link: :nonexistent/, fn ->
        Kinematics.forward_kinematics(robot, %{}, :nonexistent)
      end
    end
  end
end
