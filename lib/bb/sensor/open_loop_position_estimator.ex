# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.OpenLoopPositionEstimator do
  @moduledoc """
  A "sensor" that estimates joint position for open-loop control systems.

  This sensor subscribes to `BB.Message.Actuator.BeginMotion` messages from a
  paired actuator and uses easing functions to estimate the current joint
  position during motion. It publishes `BB.Message.Sensor.JointState` messages
  at a configurable rate.

  Use this sensor with actuators that don't provide position feedback (e.g.,
  RC servos, open-loop stepper motors). The estimator works with any joint
  type (revolute, prismatic, etc.) as it operates on raw position values.

  ## Options

  - `actuator` - Name of the actuator to subscribe to (required)
  - `easing` - Easing function for position interpolation (default: `:linear`)
  - `publish_rate` - Rate to publish position updates during motion (default: 50 Hz)
  - `max_silence` - Maximum time between publishes when idle (default: 5 seconds)

  ## Easing Functions

  The following easing functions are available (see [easings.net](https://easings.net)
  for visualisations):

  - `:linear` - Constant velocity (default)
  - `:ease_in_sine`, `:ease_out_sine`, `:ease_in_out_sine` - Sinusoidal
  - `:ease_in_quad`, `:ease_out_quad`, `:ease_in_out_quad` - Quadratic
  - `:ease_in_cubic`, `:ease_out_cubic`, `:ease_in_out_cubic` - Cubic
  - `:ease_in_quartic`, `:ease_out_quartic`, `:ease_in_out_quartic` - Quartic
  - `:ease_in_quintic`, `:ease_out_quintic`, `:ease_in_out_quintic` - Quintic
  - `:ease_in_expo`, `:ease_out_expo`, `:ease_in_out_expo` - Exponential
  - `:ease_in_circular`, `:ease_out_circular`, `:ease_in_out_circular` - Circular

  ## Example DSL Usage

      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator,
          actuator: :servo,
          easing: :ease_in_out_quad
        }
      end

  ## How It Works

  1. Subscribes to `BeginMotion` messages from the named actuator
  2. When motion begins, captures initial position, target, and expected arrival time
  3. Ticks at publish_rate during animation, interpolating position with easing
  4. Uses GenServer timeout for heartbeat publishes when idle
  5. Ensures final position is published even under system load
  """

  @easing_functions [
    :linear,
    :ease_in_sine,
    :ease_out_sine,
    :ease_in_out_sine,
    :ease_in_quad,
    :ease_out_quad,
    :ease_in_out_quad,
    :ease_in_cubic,
    :ease_out_cubic,
    :ease_in_out_cubic,
    :ease_in_quartic,
    :ease_out_quartic,
    :ease_in_out_quartic,
    :ease_in_quintic,
    :ease_out_quintic,
    :ease_in_out_quintic,
    :ease_in_expo,
    :ease_out_expo,
    :ease_in_out_expo,
    :ease_in_circular,
    :ease_out_circular,
    :ease_in_out_circular
  ]
  use BB.Sensor
  import BB.Unit
  import BB.Unit.Option

  alias BB.Cldr.Unit, as: CldrUnit
  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Sensor.JointState
  alias BB.Robot.Units

  @impl BB.Sensor
  def options_schema do
    Spark.Options.new!(
      actuator: [
        type: :atom,
        doc: "Name of the actuator to subscribe to",
        required: true
      ],
      easing: [
        type: {:in, @easing_functions},
        doc: "Easing function for position interpolation",
        default: :linear
      ],
      publish_rate: [
        type: unit_type(compatible: :hertz),
        doc: "Rate at which to publish position changes during motion",
        default: ~u(50 hertz)
      ],
      max_silence: [
        type: unit_type(compatible: :second),
        doc: "Maximum time between publishes when idle (heartbeat)",
        default: ~u(5 second)
      ]
    )
  end

  @impl GenServer
  def init(opts) do
    {:ok, state} = build_state(opts)
    BB.subscribe(state.bb.robot, [:actuator | state.actuator_path])
    {:ok, state, state.max_silence_ms}
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)

    easing = Map.get(opts, :easing, :linear)
    publish_rate = Map.get(opts, :publish_rate, ~u(50 hertz))
    max_silence = Map.get(opts, :max_silence, ~u(5 second))

    publish_interval_ms =
      publish_rate
      |> CldrUnit.convert!(:hertz)
      |> Units.extract_float()
      |> then(&round(1000 / &1))

    max_silence_ms =
      max_silence
      |> CldrUnit.convert!(:second)
      |> Units.extract_float()
      |> then(&round(&1 * 1000))

    actuator_path = build_actuator_path(opts.bb.path, opts.actuator)

    state = %{
      bb: opts.bb,
      actuator: opts.actuator,
      actuator_path: actuator_path,
      easing: easing,
      publish_interval_ms: publish_interval_ms,
      max_silence_ms: max_silence_ms,
      name: name,
      joint_name: joint_name,
      initial_position: nil,
      target_position: nil,
      expected_arrival: nil,
      command_time: nil,
      last_published: nil,
      tick_ref: nil
    }

    {:ok, state}
  end

  defp build_actuator_path(sensor_path, actuator_name) do
    [_sensor_name, joint_name | rest] = Enum.reverse(sensor_path)
    Enum.reverse([actuator_name, joint_name | rest])
  end

  @impl GenServer
  def handle_info(%Message{payload: %BeginMotion{} = cmd}, state) do
    state = cancel_tick(state)
    now = System.monotonic_time(:millisecond)

    state = %{
      state
      | initial_position: cmd.initial_position,
        target_position: cmd.target_position,
        expected_arrival: cmd.expected_arrival,
        command_time: now
    }

    state =
      if cmd.expected_arrival > now do
        schedule_tick(state)
      else
        publish_position(state, cmd.target_position)
      end

    {:noreply, state, state.max_silence_ms}
  end

  def handle_info(:tick, %{tick_ref: nil} = state) do
    {:noreply, state, state.max_silence_ms}
  end

  def handle_info(:tick, state) do
    now = System.monotonic_time(:millisecond)

    state =
      if now >= state.expected_arrival do
        state
        |> publish_position(state.target_position)
        |> Map.put(:tick_ref, nil)
      else
        position = interpolate_position(state, now)

        state
        |> maybe_publish(position)
        |> schedule_tick()
      end

    {:noreply, state, state.max_silence_ms}
  end

  def handle_info(:timeout, state) do
    state =
      if state.target_position do
        publish_position(state, current_position(state))
      else
        state
      end

    {:noreply, state, state.max_silence_ms}
  end

  defp current_position(%{target_position: nil}), do: nil

  defp current_position(state) do
    now = System.monotonic_time(:millisecond)

    if now >= state.expected_arrival do
      state.target_position
    else
      interpolate_position(state, now)
    end
  end

  defp interpolate_position(state, now) do
    total_duration = state.expected_arrival - state.command_time

    if total_duration <= 0 do
      state.target_position
    else
      elapsed = now - state.command_time
      change = state.target_position - state.initial_position

      apply(Ease, state.easing, [elapsed, state.initial_position, change, total_duration])
    end
  end

  defp maybe_publish(state, position) when position == state.last_published, do: state
  defp maybe_publish(state, position), do: publish_position(state, position)

  defp publish_position(state, position) do
    message =
      Message.new!(JointState, state.name, names: [state.joint_name], positions: [position])

    BB.publish(state.bb.robot, [:sensor | state.bb.path], message)
    %{state | last_published: position}
  end

  defp schedule_tick(state) do
    ref = Process.send_after(self(), :tick, state.publish_interval_ms)
    %{state | tick_ref: ref}
  end

  defp cancel_tick(%{tick_ref: nil} = state), do: state

  defp cancel_tick(%{tick_ref: ref} = state) do
    Process.cancel_timer(ref)
    %{state | tick_ref: nil}
  end
end
