# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.Actuator do
  @moduledoc """
  Simulated actuator for kinematic simulation mode.

  This actuator is automatically used in place of real actuators when the robot
  is started with `simulation: :kinematic`. It:

  - Receives position commands via pubsub, cast, and call
  - Calculates motion timing from joint velocity limits
  - Publishes `BeginMotion` messages for position estimation
  - Clamps positions to joint limits

  Works with `BB.Sensor.OpenLoopPositionEstimator` for position feedback.

  ## Example

      # Start robot in simulation mode
      MyRobot.start_link(simulation: :kinematic)

      # Commands work identically to hardware mode
      BB.Actuator.set_position(MyRobot, [:base, :shoulder, :motor], 1.57)
  """

  use BB.Actuator, options_schema: []

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.PubSub
  alias BB.Robot

  defstruct [:bb, :joint, :current_position, :name, :joint_name]

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    [name, joint_name | _] = Enum.reverse(bb.path)
    robot = bb.robot.robot()

    joint = Robot.get_joint(robot, joint_name)

    initial_position = calculate_initial_position(joint)

    state = %__MODULE__{
      bb: bb,
      joint: joint,
      current_position: initial_position,
      name: name,
      joint_name: joint_name
    }

    {:ok, state}
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    {:noreply, do_set_position(cmd.position, cmd.command_id, state)}
  end

  def handle_info({:bb, _path, %Message{payload: %Command.Stop{}}}, state) do
    {:noreply, state}
  end

  def handle_info({:bb, _path, %Message{payload: %Command.Hold{}}}, state) do
    {:noreply, state}
  end

  def handle_info({:bb, _path, _message}, state) do
    {:noreply, state}
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    {:noreply, do_set_position(cmd.position, cmd.command_id, state)}
  end

  def handle_cast({:command, _message}, state) do
    {:noreply, state}
  end

  @impl BB.Actuator
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    new_state = do_set_position(cmd.position, cmd.command_id, state)
    {:reply, {:ok, :accepted}, new_state}
  end

  def handle_call({:command, _message}, _from, state) do
    {:reply, {:ok, :accepted}, state}
  end

  defp do_set_position(target_position, command_id, state) do
    clamped = clamp_position(target_position, state.joint)
    travel_time_ms = calculate_travel_time(state.current_position, clamped, state.joint)
    expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

    message_opts = [
      initial_position: state.current_position,
      target_position: clamped,
      expected_arrival: expected_arrival,
      command_type: :position
    ]

    message_opts =
      if command_id do
        Keyword.put(message_opts, :command_id, command_id)
      else
        message_opts
      end

    {:ok, message} = Message.new(BeginMotion, state.joint_name, message_opts)
    PubSub.publish(state.bb.robot, [:actuator | state.bb.path], message)

    %{state | current_position: clamped}
  end

  defp calculate_initial_position(nil), do: 0.0

  defp calculate_initial_position(%{limits: nil}), do: 0.0

  defp calculate_initial_position(%{limits: limits}) do
    lower = limits.lower || 0.0
    upper = limits.upper || 0.0
    (lower + upper) / 2
  end

  defp clamp_position(position, nil), do: position
  defp clamp_position(position, %{limits: nil}), do: position

  defp clamp_position(position, %{limits: limits}) do
    position
    |> clamp_lower(limits.lower)
    |> clamp_upper(limits.upper)
  end

  defp clamp_lower(position, nil), do: position
  defp clamp_lower(position, lower), do: max(position, lower)

  defp clamp_upper(position, nil), do: position
  defp clamp_upper(position, upper), do: min(position, upper)

  defp calculate_travel_time(_from, _to, nil), do: 0
  defp calculate_travel_time(_from, _to, %{limits: nil}), do: 0
  defp calculate_travel_time(_from, _to, %{limits: %{velocity: nil}}), do: 0

  defp calculate_travel_time(from, to, %{limits: %{velocity: velocity}}) do
    distance = abs(to - from)
    round(distance / velocity * 1000)
  end
end
