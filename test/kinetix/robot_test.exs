defmodule Kinetix.RobotTest do
  use ExUnit.Case, async: true
  import Kinetix.Unit

  alias Kinetix.Robot
  alias Kinetix.Robot.{Joint, Kinematics, Link, State, Topology, Transform}

  defmodule SimpleArm do
    use Kinetix

    robot do
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
            z(~u(1 meter))
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
                z(~u(1 meter))
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
                    z(~u(1 meter))
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

    test "get_link/2 returns correct link" do
      robot = SimpleArm.robot()
      link = Robot.get_link(robot, :upper_arm)
      assert %Link{name: :upper_arm} = link
    end

    test "get_joint/2 returns correct joint" do
      robot = SimpleArm.robot()
      joint = Robot.get_joint(robot, :elbow)
      assert %Joint{name: :elbow, type: :revolute} = joint
    end

    test "parent_joint/2 returns parent joint of a link" do
      robot = SimpleArm.robot()
      assert Robot.parent_joint(robot, :base) == nil
      assert %Joint{name: :shoulder} = Robot.parent_joint(robot, :upper_arm)
      assert %Joint{name: :elbow} = Robot.parent_joint(robot, :forearm)
    end

    test "child_joints/2 returns child joints of a link" do
      robot = SimpleArm.robot()
      [shoulder] = Robot.child_joints(robot, :base)
      assert shoulder.name == :shoulder

      assert Robot.child_joints(robot, :end_effector) == []
    end

    test "path_to/2 returns path from root" do
      robot = SimpleArm.robot()
      assert Robot.path_to(robot, :base) == [:base]
      assert Robot.path_to(robot, :shoulder) == [:base, :shoulder]
      assert Robot.path_to(robot, :upper_arm) == [:base, :shoulder, :upper_arm]

      assert Robot.path_to(robot, :end_effector) == [
               :base,
               :shoulder,
               :upper_arm,
               :elbow,
               :forearm,
               :wrist,
               :end_effector
             ]
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
      upper_arm = Robot.get_link(robot, :upper_arm)

      assert upper_arm.parent_joint == :shoulder
      assert upper_arm.child_joints == [:elbow]
    end

    test "root link has no parent" do
      robot = SimpleArm.robot()
      base = Robot.get_link(robot, :base)

      assert base.parent_joint == nil
      assert base.child_joints == [:shoulder]
    end

    test "leaf link has no children" do
      robot = SimpleArm.robot()
      end_effector = Robot.get_link(robot, :end_effector)

      assert end_effector.child_joints == []
    end

    test "mass is converted to kilograms" do
      robot = SimpleArm.robot()
      base = Robot.get_link(robot, :base)

      assert base.mass == 5.0
    end
  end

  describe "Joint struct" do
    test "has parent and child link references" do
      robot = SimpleArm.robot()
      elbow = Robot.get_joint(robot, :elbow)

      assert elbow.parent_link == :upper_arm
      assert elbow.child_link == :forearm
    end

    test "origin is converted to meters and radians" do
      robot = SimpleArm.robot()
      shoulder = Robot.get_joint(robot, :shoulder)

      assert shoulder.origin.position == {0.0, 0.0, 0.1}
      assert shoulder.origin.orientation == {0.0, 0.0, 0.0}
    end

    test "axis is normalised" do
      robot = SimpleArm.robot()
      shoulder = Robot.get_joint(robot, :shoulder)

      assert shoulder.axis == {0.0, 0.0, 1.0}
    end

    test "limits are converted to radians" do
      robot = SimpleArm.robot()
      shoulder = Robot.get_joint(robot, :shoulder)

      assert_in_delta shoulder.limits.lower, -:math.pi() / 2, 0.001
      assert_in_delta shoulder.limits.upper, :math.pi() / 2, 0.001
    end

    test "velocity is converted to radians per second" do
      robot = SimpleArm.robot()
      shoulder = Robot.get_joint(robot, :shoulder)

      expected = 2.0 * :math.pi() / 180.0
      assert_in_delta shoulder.limits.velocity, expected, 0.0001
    end

    test "rotational?/1 returns true for revolute joints" do
      robot = SimpleArm.robot()
      shoulder = Robot.get_joint(robot, :shoulder)
      assert Joint.rotational?(shoulder)
    end

    test "movable?/1 returns true for non-fixed joints" do
      robot = SimpleArm.robot()
      shoulder = Robot.get_joint(robot, :shoulder)
      assert Joint.movable?(shoulder)
    end
  end

  describe "Topology struct" do
    test "depth_of/2 returns correct depth" do
      robot = SimpleArm.robot()
      topology = robot.topology

      assert Topology.depth_of(topology, :base) == 0
      assert Topology.depth_of(topology, :shoulder) == 1
      assert Topology.depth_of(topology, :upper_arm) == 1
      assert Topology.depth_of(topology, :elbow) == 2
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
      assert Nx.shape(t) == {4, 4}

      expected = Nx.eye(4, type: :f64)
      assert Nx.to_list(t) == Nx.to_list(expected)
    end

    test "translation/3 creates translation matrix" do
      t = Transform.translation(1.0, 2.0, 3.0)
      assert Transform.get_translation(t) == {1.0, 2.0, 3.0}
    end

    test "rotation_x/1 rotates around X axis" do
      t = Transform.rotation_x(:math.pi() / 2)
      {x, y, z} = Transform.apply_to_point(t, {0.0, 1.0, 0.0})

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 1.0, 0.0001
    end

    test "rotation_y/1 rotates around Y axis" do
      t = Transform.rotation_y(:math.pi() / 2)
      {x, y, z} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, -1.0, 0.0001
    end

    test "rotation_z/1 rotates around Z axis" do
      t = Transform.rotation_z(:math.pi() / 2)
      {x, y, z} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 1.0, 0.0001
      assert_in_delta z, 0.0, 0.0001
    end

    test "compose/2 multiplies transforms" do
      t1 = Transform.translation(1.0, 0.0, 0.0)
      t2 = Transform.translation(0.0, 2.0, 0.0)
      t = Transform.compose(t1, t2)

      assert Transform.get_translation(t) == {1.0, 2.0, 0.0}
    end

    test "compose_all/1 composes multiple transforms" do
      transforms = [
        Transform.translation(1.0, 0.0, 0.0),
        Transform.translation(0.0, 1.0, 0.0),
        Transform.translation(0.0, 0.0, 1.0)
      ]

      t = Transform.compose_all(transforms)
      assert Transform.get_translation(t) == {1.0, 1.0, 1.0}
    end

    test "inverse/1 computes inverse transform" do
      t = Transform.translation(1.0, 2.0, 3.0)
      t_inv = Transform.inverse(t)

      {x, y, z} = Transform.get_translation(t_inv)
      assert_in_delta x, -1.0, 0.0001
      assert_in_delta y, -2.0, 0.0001
      assert_in_delta z, -3.0, 0.0001
    end

    test "revolute_transform/2 rotates around arbitrary axis" do
      axis = {0.0, 0.0, 1.0}
      t = Transform.revolute_transform(axis, :math.pi() / 2)
      {x, y, _z} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 1.0, 0.0001
    end

    test "prismatic_transform/2 translates along axis" do
      axis = {1.0, 0.0, 0.0}
      t = Transform.prismatic_transform(axis, 2.5)

      assert Transform.get_translation(t) == {2.5, 0.0, 0.0}
    end

    test "from_origin/1 creates transform from position and orientation" do
      origin = %{
        position: {1.0, 2.0, 3.0},
        orientation: {0.0, 0.0, 0.0}
      }

      t = Transform.from_origin(origin)
      assert Transform.get_translation(t) == {1.0, 2.0, 3.0}
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
      {x, y, z} = Transform.get_translation(transform)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 1.0, 0.0001

      State.delete(state)
    end

    test "forward_kinematics/3 with joint position" do
      robot = SimpleArm.robot()
      {:ok, state} = State.new(robot)

      State.set_joint_position(state, :shoulder, :math.pi() / 2)

      transform = Kinematics.forward_kinematics(robot, state, :upper_arm)
      {x, y, z} = Transform.get_translation(transform)

      assert_in_delta x, 0.0, 0.0001
      assert_in_delta y, 0.0, 0.0001
      assert_in_delta z, 0.1, 0.0001

      State.delete(state)
    end

    test "forward_kinematics/3 accepts position map" do
      robot = SimpleArm.robot()
      positions = %{shoulder: 0.0, elbow: 0.0, wrist: 0.0}

      transform = Kinematics.forward_kinematics(robot, positions, :end_effector)
      {_x, _y, z} = Transform.get_translation(transform)

      assert_in_delta z, 1.0, 0.0001
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
      {x, y, z} = Transform.get_translation(base_transform)
      assert {x, y, z} == {0.0, 0.0, 0.0}

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

      assert_raise ArgumentError, ~r/Unknown link/, fn ->
        Kinematics.forward_kinematics(robot, %{}, :nonexistent)
      end
    end
  end
end
