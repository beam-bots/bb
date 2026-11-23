# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.StateHistoryTest do
  use ExUnit.Case, async: true

  alias BB.Error.State.JointHistoryDisabled
  alias BB.Error.State.OutsideJointHistory
  alias BB.ExampleRobots.DifferentialDriveRobot
  alias BB.ExampleRobots.FloatingDrone
  alias BB.ExampleRobots.PlanarRover
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Robot.State

  @second 1_000_000_000
  @history %{keep: 10 * @second, interpolation?: true, max_gap: @second}

  # Creating a state records every joint's initial configuration at the moment
  # of creation. Samples here are placed relative to a time read *before* that,
  # so each is strictly older than the initial sample and every lookup at or
  # before it is in the past - whatever the scheduler does, and however long the
  # test takes.
  #
  # A lookup resolves every joint, and one the test doesn't write would have
  # nothing older than creation, so each joint also gets a baseline sample a
  # minute before.
  defp state_with_history(robot_module, history \\ @history) do
    before = System.monotonic_time(:nanosecond)
    {:ok, state} = State.new(robot_module.robot(), history: history)

    :ok =
      State.set_configurations(state, State.get_all_configurations(state), baseline(before))

    {state, before}
  end

  defp baseline(before), do: before - 60 * @second

  defp mast_at(state, time) do
    {:ok, configurations} = State.get_all_configurations_at(state, time)
    configurations.mast
  end

  describe "a state without history" do
    test "refuses lookups" do
      {:ok, state} = State.new(PlanarRover.robot())

      assert {:error, %JointHistoryDisabled{}} =
               State.get_all_configurations_at(state, System.monotonic_time(:nanosecond))
    end
  end

  describe "looking up a revolute joint" do
    setup do
      {state, before} = state_with_history(PlanarRover)
      earlier = before - 3 * @second
      :ok = State.set_configuration(state, :mast, 0.2, earlier)
      :ok = State.set_configuration(state, :mast, 0.6, earlier + 400_000_000)
      %{state: state, before: before, earlier: earlier}
    end

    test "returns a sample exactly", %{state: state, earlier: earlier} do
      assert mast_at(state, earlier) == 0.2
    end

    test "interpolates between samples within max_gap", %{state: state, earlier: earlier} do
      assert_in_delta mast_at(state, earlier + 100_000_000), 0.3, 1.0e-12
    end

    test "holds the earlier sample across a gap wider than max_gap", %{
      state: state,
      before: before
    } do
      # The next sample is the initial one recorded at creation, which is at
      # least 2.6 seconds after 0.6 was written.
      assert mast_at(state, before - 2 * @second) == 0.6
    end

    test "refuses a time before the joint's oldest sample", %{state: state, before: before} do
      oldest = baseline(before)
      requested = oldest - 1

      assert {:error,
              %OutsideJointHistory{
                direction: :past,
                requested_time: ^requested,
                boundary_time: ^oldest
              }} = State.get_all_configurations_at(state, requested)
    end

    test "refuses a time later than now", %{state: state} do
      requested = System.monotonic_time(:nanosecond) + 60 * @second

      assert {:error,
              %OutsideJointHistory{direction: :future, joint: nil, requested_time: ^requested}} =
               State.get_all_configurations_at(state, requested)
    end
  end

  describe "after a joint's newest sample" do
    test "holds the newest sample" do
      {state, _before} = state_with_history(PlanarRover)
      :ok = State.set_configuration(state, :mast, 0.9, System.monotonic_time(:nanosecond))

      assert mast_at(state, System.monotonic_time(:nanosecond)) == 0.9
    end
  end

  describe "with interpolation off" do
    test "returns the earlier sample" do
      {state, before} = state_with_history(PlanarRover, %{@history | interpolation?: false})
      earlier = before - 3 * @second
      :ok = State.set_configuration(state, :mast, 0.2, earlier)
      :ok = State.set_configuration(state, :mast, 0.6, earlier + 400_000_000)

      assert mast_at(state, earlier + 100_000_000) == 0.2
    end
  end

  describe "writes" do
    test "a second write at the same time replaces the first" do
      {state, before} = state_with_history(PlanarRover)
      :ok = State.set_configuration(state, :mast, 0.2, before - @second)
      :ok = State.set_configuration(state, :mast, 0.4, before - @second)

      assert mast_at(state, before - @second) == 0.4
    end

    test "arriving out of order are placed in time order" do
      {state, before} = state_with_history(PlanarRover)
      earlier = before - 3 * @second
      :ok = State.set_configuration(state, :mast, 0.6, earlier + 400_000_000)
      :ok = State.set_configuration(state, :mast, 0.2, earlier)

      assert_in_delta mast_at(state, earlier + 200_000_000), 0.4, 1.0e-12
    end

    test "set_configurations/3 records every joint at the given time" do
      {state, before} = state_with_history(PlanarRover)
      base = Transform2D.new(1.0, 2.0, 0.5)

      :ok = State.set_configurations(state, %{mast: 0.7, base: base}, before - @second)

      assert {:ok, %{mast: 0.7, base: ^base}} =
               State.get_all_configurations_at(state, before - @second)
    end

    test "rejected writes record nothing" do
      {state, before} = state_with_history(PlanarRover)

      assert {:error, _} =
               State.set_configuration(state, :mast, Transform.identity(), before - 1)

      assert mast_at(state, before - 1) == 0.0
    end
  end

  describe "pruning" do
    setup do
      {state, before} = state_with_history(PlanarRover, %{@history | keep: @second})
      :ok = State.set_configuration(state, :mast, 0.1, before - 30 * @second)
      :ok = State.set_configuration(state, :mast, 0.2, before - 29 * @second)
      :ok = State.set_configuration(state, :mast, 0.3, before - 20 * @second)
      %{state: state, before: before}
    end

    test "discards samples older than keep", %{state: state, before: before} do
      retained = before - 29 * @second

      assert {:error, %OutsideJointHistory{direction: :past, boundary_time: ^retained}} =
               State.get_all_configurations_at(state, before - 30 * @second)
    end

    test "keeps the newest sample before the cutoff", %{state: state, before: before} do
      assert mast_at(state, before - 25 * @second) == 0.2
    end
  end

  describe "joint types" do
    test "fixed joints are present and always zero" do
      {state, _before} = state_with_history(DifferentialDriveRobot)

      assert {:ok, %{caster_joint: +0.0}} =
               State.get_all_configurations_at(state, System.monotonic_time(:nanosecond))
    end

    test "continuous joints interpolate along the shorter arc" do
      {state, before} = state_with_history(DifferentialDriveRobot)
      earlier = before - 3 * @second
      :ok = State.set_configuration(state, :left_wheel_joint, 3.0, earlier)
      :ok = State.set_configuration(state, :left_wheel_joint, -3.0, earlier + 400_000_000)

      {:ok, %{left_wheel_joint: angle}} =
        State.get_all_configurations_at(state, earlier + 200_000_000)

      assert_in_delta abs(angle), :math.pi(), 1.0e-9
    end

    test "planar joints interpolate translation and heading" do
      {state, before} = state_with_history(PlanarRover)
      earlier = before - 3 * @second
      :ok = State.set_configuration(state, :base, Transform2D.new(0.0, 0.0, 0.0), earlier)

      :ok =
        State.set_configuration(
          state,
          :base,
          Transform2D.new(2.0, 4.0, 1.0),
          earlier + 400_000_000
        )

      {:ok, %{base: base}} = State.get_all_configurations_at(state, earlier + 100_000_000)

      assert_in_delta base.x, 0.5, 1.0e-12
      assert_in_delta base.y, 1.0, 1.0e-12
      assert_in_delta base.theta, 0.25, 1.0e-12
    end

    test "floating joints interpolate translation linearly and rotation by slerp" do
      {state, before} = state_with_history(FloatingDrone)
      earlier = before - 3 * @second

      :ok = State.set_configuration(state, :base, Transform.identity(), earlier)

      target =
        Transform.from_position_quaternion(
          Vec3.new(4.0, 0.0, 0.0),
          Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 2)
        )

      :ok = State.set_configuration(state, :base, target, earlier + 400_000_000)

      {:ok, %{base: base}} = State.get_all_configurations_at(state, earlier + 200_000_000)
      {_axis, angle} = base |> Transform.get_quaternion() |> Quaternion.to_axis_angle()

      assert_in_delta Vec3.x(Transform.get_translation(base)), 2.0, 1.0e-9
      assert_in_delta angle, :math.pi() / 4, 1.0e-9
    end
  end
end
