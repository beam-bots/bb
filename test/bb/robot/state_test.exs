# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.StateTest do
  use ExUnit.Case, async: true

  alias BB.Error.Invalid.JointConfig
  alias BB.ExampleRobots.DifferentialDriveRobot
  alias BB.ExampleRobots.FloatingDrone
  alias BB.ExampleRobots.PlanarRover
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Message.Geometry.Twist
  alias BB.Message.Geometry.Twist2D
  alias BB.Robot.State

  # Deliberately not axis-aligned and not round, so a value that silently goes
  # through a quaternion decomposition or a float32 hop shows up as a mismatch.
  defp awkward_transform do
    Transform.compose(
      Transform.translation(Vec3.new(12.400000000000001, -3.0999999999999996, 0.7)),
      Transform.from_quaternion(
        Quaternion.from_axis_angle(
          Vec3.normalise(Vec3.new(0.3, -0.7, 0.64)),
          1.2345678901234567
        )
      )
    )
  end

  defp planar_state do
    {:ok, state} = State.new(PlanarRover.robot())
    state
  end

  defp floating_state do
    {:ok, state} = State.new(FloatingDrone.robot())
    state
  end

  describe "planar joints" do
    test "default to the identity" do
      assert State.get_configuration(planar_state(), :base) ==
               {:ok, Transform2D.identity()}
    end

    test "round-trip a Transform2D exactly" do
      state = planar_state()
      configuration = Transform2D.new(12.4, -3.1, 1.5707963267948966)

      assert :ok = State.set_configuration(state, :base, configuration)
      assert State.get_configuration(state, :base) == {:ok, configuration}
    end

    test "round-trip a Twist2D exactly" do
      state = planar_state()
      velocity = %Twist2D{vx: 0.35, vy: -0.125, omega: 0.98765}

      assert :ok = State.set_velocity(state, :base, velocity)
      assert State.get_velocity(state, :base) == {:ok, velocity}
    end

    test "reject a bare float, naming Transform2D as expected" do
      assert {:error, %JointConfig{joint: :base, field: :configuration, expected: Transform2D}} =
               State.set_configuration(planar_state(), :base, 0.5)
    end

    test "reject a Transform, since a planar joint is not free in 3D" do
      assert {:error, %JointConfig{expected: Transform2D}} =
               State.set_configuration(planar_state(), :base, Transform.identity())
    end

    test "reject a 3D Twist as a velocity" do
      twist = %Twist{linear: Vec3.zero(), angular: Vec3.zero()}

      assert {:error, %JointConfig{field: :velocity, expected: Twist2D}} =
               State.set_velocity(planar_state(), :base, twist)
    end

    test "sit alongside single-DoF joints in get_all_configurations/1" do
      state = planar_state()
      configuration = Transform2D.new(1.0, 2.0, 0.5)

      :ok = State.set_configuration(state, :base, configuration)
      :ok = State.set_configuration(state, :mast, 0.75)

      assert State.get_all_configurations(state) == %{base: configuration, mast: 0.75}
    end

    test "appear in a chain's configurations in traversal order" do
      state = planar_state()
      configuration = Transform2D.new(1.0, 2.0, 0.5)

      :ok = State.set_configuration(state, :base, configuration)
      :ok = State.set_configuration(state, :mast, 0.75)

      assert State.get_chain_configurations(state, :sensor_head) == [
               {:base, configuration},
               {:mast, 0.75}
             ]
    end
  end

  describe "floating joints" do
    test "default to the identity" do
      {:ok, configuration} = State.get_configuration(floating_state(), :base)

      assert Nx.to_flat_list(Transform.tensor(configuration)) ==
               Nx.to_flat_list(Transform.tensor(Transform.identity()))
    end

    # The whole point of storing raw bytes rather than decomposing to a
    # quaternion and translation: 16 numbers to 7 is not a bijection, and the
    # round-trip would run on every read, so error would accumulate.
    test "round-trip a Transform bit-exactly" do
      state = floating_state()
      configuration = awkward_transform()

      assert :ok = State.set_configuration(state, :base, configuration)
      {:ok, recovered} = State.get_configuration(state, :base)

      assert Nx.to_binary(Transform.tensor(recovered)) ==
               Nx.to_binary(Transform.tensor(configuration))
    end

    test "survive repeated reads without drifting" do
      state = floating_state()
      configuration = awkward_transform()
      :ok = State.set_configuration(state, :base, configuration)

      expected = Nx.to_binary(Transform.tensor(configuration))

      for _ <- 1..25 do
        {:ok, recovered} = State.get_configuration(state, :base)
        assert Nx.to_binary(Transform.tensor(recovered)) == expected
      end
    end

    test "round-trip a Twist bit-exactly" do
      state = floating_state()

      velocity = %Twist{
        linear: Vec3.new(1.2345678901234567, -0.9876543210987654, 0.5),
        angular: Vec3.new(-0.111111111111111, 0.222222222222222, -0.333333333333333)
      }

      assert :ok = State.set_velocity(state, :base, velocity)
      {:ok, recovered} = State.get_velocity(state, :base)

      assert Nx.to_binary(Vec3.tensor(recovered.linear)) ==
               Nx.to_binary(Vec3.tensor(velocity.linear))

      assert Nx.to_binary(Vec3.tensor(recovered.angular)) ==
               Nx.to_binary(Vec3.tensor(velocity.angular))
    end

    test "keep no tensor struct in the table, so the backend can't leak in" do
      state = floating_state()
      :ok = State.set_configuration(state, :base, awkward_transform())

      assert [{{:configuration, :base}, stored}] =
               :ets.lookup(state.table, {:configuration, :base})

      assert is_binary(stored)
      assert byte_size(stored) == 128
    end

    test "reject a bare float, naming Transform as expected" do
      assert {:error, %JointConfig{joint: :base, expected: Transform}} =
               State.set_configuration(floating_state(), :base, 0.5)
    end

    test "reject a Transform2D, since a floating joint is not confined to a plane" do
      assert {:error, %JointConfig{expected: Transform}} =
               State.set_configuration(floating_state(), :base, Transform2D.identity())
    end

    test "reject a Twist2D as a velocity" do
      velocity = %Twist2D{vx: 0.0, vy: 0.0, omega: 0.0}

      assert {:error, %JointConfig{field: :velocity, expected: Twist}} =
               State.set_velocity(floating_state(), :base, velocity)
    end
  end

  describe "fixed joints" do
    setup do
      {:ok, state} = State.new(DifferentialDriveRobot.robot())

      %{state: state}
    end

    test "read as zero, which is what forward kinematics expects", %{state: state} do
      assert State.get_configuration(state, :caster_joint) == {:ok, 0.0}
      assert Map.fetch!(State.get_all_configurations(state), :caster_joint) == 0.0
    end

    # Zero DoF means a configuration space with exactly one point, so that point
    # is assignable. Rejecting it broke read-modify-write on any robot with a
    # fixed joint, which is most of them.
    test "accept the single configuration they have", %{state: state} do
      assert :ok = State.set_configuration(state, :caster_joint, 0.0)
      assert :ok = State.set_configuration(state, :caster_joint, 0)
      assert State.get_configuration(state, :caster_joint) == {:ok, 0.0}
    end

    test "reject a value they could never take", %{state: state} do
      assert {:error, %JointConfig{joint: :caster_joint, expected: +0.0}} =
               State.set_configuration(state, :caster_joint, 0.5)

      assert {:error, %JointConfig{joint: :caster_joint}} =
               State.set_configuration(state, :caster_joint, Transform.identity())
    end
  end

  describe "read-modify-write" do
    # get_all_configurations/1 must hand back a map set_configurations/2 accepts,
    # or the most obvious way to use this module is broken.
    test "a whole configuration map round-trips unchanged" do
      for robot <- [
            DifferentialDriveRobot.robot(),
            PlanarRover.robot(),
            FloatingDrone.robot()
          ] do
        {:ok, state} = State.new(robot)

        configurations = State.get_all_configurations(state)
        assert :ok = State.set_configurations(state, configurations)
      end
    end

    test "a whole velocity map round-trips unchanged" do
      for robot <- [
            DifferentialDriveRobot.robot(),
            PlanarRover.robot(),
            FloatingDrone.robot()
          ] do
        {:ok, state} = State.new(robot)

        assert :ok = State.set_velocities(state, State.get_all_velocities(state))
      end
    end

    test "round-trips a modified map, including the multi-DoF entries" do
      {:ok, state} = State.new(PlanarRover.robot())
      configuration = Transform2D.new(1.5, -2.5, 0.75)

      modified =
        state
        |> State.get_all_configurations()
        |> Map.put(:base, configuration)
        |> Map.put(:mast, 0.25)

      assert :ok = State.set_configurations(state, modified)
      assert State.get_all_configurations(state) == modified
    end
  end

  describe "bulk writes" do
    test "accept a mixture of shapes" do
      state = planar_state()
      configuration = Transform2D.new(3.0, 4.0, -0.25)

      assert :ok = State.set_configurations(state, %{base: configuration, mast: 0.5})

      assert State.get_configuration(state, :base) == {:ok, configuration}
      assert State.get_configuration(state, :mast) == {:ok, 0.5}
    end

    test "reject the whole map when one shape is wrong" do
      state = planar_state()

      assert {:error, %JointConfig{joint: :base}} =
               State.set_configurations(state, %{base: 0.5, mast: 0.5})

      assert State.get_configuration(state, :base) == {:ok, Transform2D.identity()}
      assert State.get_configuration(state, :mast) == {:ok, 0.0}
    end
  end
end
