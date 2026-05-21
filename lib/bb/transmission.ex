# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Transmission do
  @moduledoc """
  Mechanical transmission math: convert quantities between joint-space and
  motor-space.

  A transmission captures three properties of the linkage between a joint
  and its actuator:

  - `reduction` — the gear ratio. A reduction of `n` means the actuator
    rotates `n` times for one rotation of the joint, so motor angular
    motion is `n` times the joint motion, and motor torque is `1 / n`
    times the joint torque.
  - `offset` — the joint-space value (radians for rotational joints, metres
    for linear joints) corresponding to the actuator's zero position.
    Applies to positions only — velocities and accelerations have no
    offset since they are rates.
  - `reversed?` — whether actuator motion is reversed relative to joint
    motion.

  All values are floats in SI base units (radians, metres, rad/s, m/s,
  N·m, N).

  ## Equations

  Position (joint → motor):       `motor = sign × reduction × (joint − offset)`
  Position (motor → joint):       `joint = sign × motor / reduction + offset`
  Rate (velocity, acceleration):  `motor = sign × reduction × joint`
  Effort (torque, force):         `motor = sign × joint / reduction`

  Where `sign = -1` if `reversed?`, otherwise `+1`. Each pair of
  apply/unapply is an exact inverse within float precision.
  """

  @type t :: %{
          required(:reduction) => float,
          required(:offset) => float,
          required(:reversed?) => boolean
        }

  @doc "Convert a joint-space position into a motor-space position."
  @spec apply_position(float, t) :: float
  def apply_position(value, %{reduction: r, offset: o, reversed?: rev}),
    do: sign(rev) * r * (value - o)

  @doc "Convert a motor-space position into a joint-space position."
  @spec unapply_position(float, t) :: float
  def unapply_position(value, %{reduction: r, offset: o, reversed?: rev}),
    do: sign(rev) * value / r + o

  @doc """
  Convert a joint-space rate (velocity or acceleration) into motor-space.

  No offset is applied since the offset is a constant in position space.
  """
  @spec apply_rate(float, t) :: float
  def apply_rate(value, %{reduction: r, reversed?: rev}),
    do: sign(rev) * r * value

  @doc "Convert a motor-space rate into a joint-space rate."
  @spec unapply_rate(float, t) :: float
  def unapply_rate(value, %{reduction: r, reversed?: rev}),
    do: sign(rev) * value / r

  @doc """
  Convert a joint-space effort (torque for rotational, force for linear)
  into motor-space.

  A gear reduction multiplies position and velocity but divides effort:
  a 50:1 reduction means the motor needs `1 / 50` of the joint torque.
  """
  @spec apply_effort(float, t) :: float
  def apply_effort(value, %{reduction: r, reversed?: rev}),
    do: sign(rev) * value / r

  @doc "Convert a motor-space effort into a joint-space effort."
  @spec unapply_effort(float, t) :: float
  def unapply_effort(value, %{reduction: r, reversed?: rev}),
    do: sign(rev) * value * r

  defp sign(true), do: -1.0
  defp sign(false), do: 1.0

  # ----------------------------------------------------------------------------
  # Message-level helpers
  # ----------------------------------------------------------------------------

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Message.Sensor.JointState

  @doc """
  Apply a transmission to an actuator command message, returning a new
  message with the payload values transformed into motor-space.

  Position, velocity, and effort commands are transformed; hold and stop
  commands have no values to transform and are returned unchanged.
  Trajectory waypoints are transformed pointwise.

  When `transmission` is `nil`, the message is returned unchanged.
  """
  @spec apply_to_command(Message.t(), t() | nil) :: Message.t()
  def apply_to_command(message, nil), do: message

  def apply_to_command(%Message{payload: payload} = message, transmission) do
    %{message | payload: apply_to_payload(payload, transmission)}
  end

  defp apply_to_payload(%Command.Position{} = cmd, t) do
    %{
      cmd
      | position: apply_position(cmd.position, t),
        velocity: maybe_apply_rate(cmd.velocity, t)
    }
  end

  defp apply_to_payload(%Command.Velocity{velocity: v} = cmd, t) do
    %{cmd | velocity: apply_rate(v, t)}
  end

  defp apply_to_payload(%Command.Effort{effort: e} = cmd, t) do
    %{cmd | effort: apply_effort(e, t)}
  end

  defp apply_to_payload(%Command.Trajectory{waypoints: waypoints} = cmd, t) do
    %{cmd | waypoints: Enum.map(waypoints, &apply_to_waypoint(&1, t))}
  end

  defp apply_to_payload(other, _t), do: other

  defp apply_to_waypoint(%{position: p, velocity: v, acceleration: a} = wp, t) do
    %{
      wp
      | position: apply_position(p, t),
        velocity: apply_rate(v, t),
        acceleration: apply_rate(a, t)
    }
  end

  defp maybe_apply_rate(nil, _t), do: nil
  defp maybe_apply_rate(value, t), do: apply_rate(value, t)

  @doc """
  Unapply a transmission to a message published by an actuator, returning a
  new message with motor-space values transformed back into joint-space.

  `BeginMotion` and `JointState` payloads are supported. `JointState`
  messages must be single-joint — the same transmission is applied to every
  list element. Other payloads are returned unchanged.

  `peak_velocity` and `acceleration` in `BeginMotion` are magnitudes, so
  their joint-space values are taken as the absolute value of the unapplied
  rate (a reversed transmission flips the sign but does not change the
  magnitude).

  When `transmission` is `nil`, the message is returned unchanged.
  """
  @spec unapply_to_payload(Message.t(), t() | nil) :: Message.t()
  def unapply_to_payload(message, nil), do: message

  def unapply_to_payload(%Message{payload: payload} = message, transmission) do
    %{message | payload: unapply_payload(payload, transmission)}
  end

  defp unapply_payload(%BeginMotion{} = msg, t) do
    %{
      msg
      | initial_position: unapply_position(msg.initial_position, t),
        target_position: unapply_position(msg.target_position, t),
        peak_velocity: maybe_unapply_rate_magnitude(msg.peak_velocity, t),
        acceleration: maybe_unapply_rate_magnitude(msg.acceleration, t)
    }
  end

  defp unapply_payload(%JointState{} = msg, t) do
    %{
      msg
      | positions: Enum.map(msg.positions, &unapply_position(&1, t)),
        velocities: Enum.map(msg.velocities, &unapply_rate(&1, t)),
        efforts: Enum.map(msg.efforts, &unapply_effort(&1, t))
    }
  end

  defp unapply_payload(other, _t), do: other

  defp maybe_unapply_rate_magnitude(nil, _t), do: nil
  defp maybe_unapply_rate_magnitude(value, t), do: abs(unapply_rate(value, t))
end
