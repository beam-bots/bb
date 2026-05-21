# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.Actuator do
  @moduledoc """
  Simulated actuator for kinematic simulation mode.

  This actuator is automatically used in place of real actuators when the robot
  is started with `simulation: :kinematic`. It:

  - Receives position commands via pubsub, cast, and call
  - Calculates motion timing from the motor profile's velocity and
    acceleration limits
  - Publishes `BeginMotion` messages (in joint-space, via
    `BB.Actuator.publish_begin_motion/3`) for position estimation
  - Clamps positions to the motor-space limits derived from the joint

  Like every other driver, the sim operates purely in motor-space: the
  wrapper transforms inbound commands joint→motor before they arrive here,
  and the outbound publish helper transforms motor→joint on the way out.
  When the joint has no transmission, motor-space and joint-space are
  identical.

  Works with `BB.Sensor.OpenLoopPositionEstimator` for position feedback.

  ## Example

      # Start robot in simulation mode
      MyRobot.start_link(simulation: :kinematic)

      # Commands work identically to hardware mode
      BB.Actuator.set_position(MyRobot, [:base, :shoulder, :motor], 1.57)
  """

  use BB.Actuator, options_schema: []

  alias BB.Message
  alias BB.Message.Actuator.Command

  defstruct [
    :bb,
    :name,
    :joint_name,
    :motor_profile,
    :current_motor_position,
    # Trajectory segment used to compute the joint's actual current position
    # (vs. its last commanded target) when a new command arrives. When
    # `segment` is nil the joint is stationary at `current_motor_position`.
    :segment
  ]

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    motor_profile = Keyword.fetch!(opts, :motor_profile)
    [name, joint_name | _] = Enum.reverse(bb.path)

    state = %__MODULE__{
      bb: bb,
      name: name,
      joint_name: joint_name,
      motor_profile: motor_profile,
      current_motor_position: motor_profile.motor_initial_position,
      segment: nil
    }

    {:ok, state}
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    {:ok, %{state | motor_profile: Keyword.fetch!(new_opts, :motor_profile)}}
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

  defp do_set_position(target_motor_position, command_id, state) do
    now = System.monotonic_time(:millisecond)
    actual_current = position_at(state, now)
    clamped = clamp_motor_position(target_motor_position, state.motor_profile)

    profile = build_profile(actual_current, clamped, state.motor_profile, now)

    message_opts = [
      initial_position: actual_current,
      target_position: clamped,
      expected_arrival: profile.expected_arrival,
      command_type: :position,
      acceleration: profile.acceleration,
      peak_velocity: profile.peak_velocity
    ]

    message_opts =
      if command_id do
        Keyword.put(message_opts, :command_id, command_id)
      else
        message_opts
      end

    BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path, message_opts)

    %{state | current_motor_position: clamped, segment: profile.segment}
  end

  # Returns the actuator's actual current position, taking into account any
  # in-flight motion segment.
  defp position_at(%{segment: nil, current_motor_position: pos}, _now), do: pos

  defp position_at(%{segment: segment}, now) do
    interpolate_segment(segment, now)
  end

  defp interpolate_segment(%{total_duration_ms: 0} = segment, _now), do: segment.target

  defp interpolate_segment(%{accel_duration_ms: 0} = segment, now) do
    # Rectangular profile fallback (no acceleration limit).
    elapsed = now - segment.command_time
    total = segment.total_duration_ms

    cond do
      elapsed <= 0 -> segment.initial
      elapsed >= total -> segment.target
      true -> segment.initial + (segment.target - segment.initial) * (elapsed / total)
    end
  end

  defp interpolate_segment(segment, now) do
    elapsed_ms = now - segment.command_time
    total_ms = segment.total_duration_ms
    accel_ms = segment.accel_duration_ms

    cond do
      elapsed_ms <= 0 ->
        segment.initial

      elapsed_ms >= total_ms ->
        segment.target

      elapsed_ms < accel_ms ->
        # Accel phase: ½ a t²
        t = elapsed_ms / 1000
        segment.initial + 0.5 * segment.signed_accel * t * t

      elapsed_ms > total_ms - accel_ms ->
        # Decel phase: mirror of accel from the end.
        t_remaining = (total_ms - elapsed_ms) / 1000
        segment.target - 0.5 * segment.signed_accel * t_remaining * t_remaining

      true ->
        # Cruise: linear at peak velocity from the end of accel.
        accel_s = accel_ms / 1000
        cruise_elapsed_s = (elapsed_ms - accel_ms) / 1000
        accel_distance = 0.5 * segment.signed_accel * accel_s * accel_s
        segment.initial + accel_distance + segment.signed_peak_velocity * cruise_elapsed_s
    end
  end

  defp build_profile(from, to, motor_profile, now) do
    velocity = motor_profile.motor_velocity_limit
    acceleration = motor_profile.motor_acceleration_limit
    distance = to - from
    abs_distance = abs(distance)
    direction = if distance >= 0, do: 1.0, else: -1.0

    cond do
      abs_distance == 0.0 ->
        %{
          expected_arrival: now,
          acceleration: nil,
          peak_velocity: nil,
          segment: %{
            initial: from,
            target: to,
            command_time: now,
            total_duration_ms: 0,
            accel_duration_ms: 0,
            signed_accel: 0.0,
            signed_peak_velocity: 0.0
          }
        }

      velocity == nil ->
        %{
          expected_arrival: now,
          acceleration: nil,
          peak_velocity: nil,
          segment: %{
            initial: from,
            target: to,
            command_time: now,
            total_duration_ms: 0,
            accel_duration_ms: 0,
            signed_accel: 0.0,
            signed_peak_velocity: 0.0
          }
        }

      acceleration == nil ->
        # Rectangular profile (existing behaviour).
        total_ms = round(abs_distance / velocity * 1000)

        %{
          expected_arrival: now + total_ms,
          acceleration: nil,
          peak_velocity: nil,
          segment: %{
            initial: from,
            target: to,
            command_time: now,
            total_duration_ms: total_ms,
            accel_duration_ms: 0,
            signed_accel: 0.0,
            signed_peak_velocity: direction * velocity
          }
        }

      true ->
        # Trapezoidal/triangular profile.
        t_accel = velocity / acceleration
        d_accel = 0.5 * acceleration * t_accel * t_accel

        {accel_s, total_s, peak_v} =
          if abs_distance >= 2 * d_accel do
            # Trapezoid: accel ramp, cruise, decel ramp.
            cruise_s = (abs_distance - 2 * d_accel) / velocity
            {t_accel, 2 * t_accel + cruise_s, velocity}
          else
            # Triangle: never reach v_max.
            peak_v = :math.sqrt(abs_distance * acceleration)
            t = peak_v / acceleration
            {t, 2 * t, peak_v}
          end

        total_ms = round(total_s * 1000)
        accel_ms = round(accel_s * 1000)

        %{
          expected_arrival: now + total_ms,
          acceleration: acceleration,
          peak_velocity: peak_v,
          segment: %{
            initial: from,
            target: to,
            command_time: now,
            total_duration_ms: total_ms,
            accel_duration_ms: accel_ms,
            signed_accel: direction * acceleration,
            signed_peak_velocity: direction * peak_v
          }
        }
    end
  end

  defp clamp_motor_position(position, %{motor_lower: nil, motor_upper: nil}), do: position

  defp clamp_motor_position(position, %{motor_lower: lower, motor_upper: upper}) do
    position
    |> clamp_lower(lower)
    |> clamp_upper(upper)
  end

  defp clamp_lower(position, nil), do: position
  defp clamp_lower(position, lower), do: max(position, lower)

  defp clamp_upper(position, nil), do: position
  defp clamp_upper(position, upper), do: min(position, upper)
end
