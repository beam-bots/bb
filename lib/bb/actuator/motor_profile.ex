# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.MotorProfile do
  @moduledoc """
  Motor-space limits and starting position for an actuator.

  Built by `BB.Actuator.Server` from a joint's limits and resolved
  transmission, then injected into the driver's resolved options as
  `:motor_profile`. Drivers read what they need from the profile instead of
  fetching the joint and applying the transmission themselves.

  Position fields are signed motor-space values; rate and effort fields are
  positive magnitudes. Any field may be `nil` if the corresponding joint
  limit is unset.

  `motor_initial_position` is the midpoint of `motor_lower` and
  `motor_upper`, falling back to `0.0` when either is `nil`.
  """

  alias BB.Transmission

  defstruct [
    :motor_lower,
    :motor_upper,
    :motor_velocity_limit,
    :motor_acceleration_limit,
    :motor_effort_limit,
    motor_initial_position: 0.0
  ]

  @type t :: %__MODULE__{
          motor_lower: float() | nil,
          motor_upper: float() | nil,
          motor_velocity_limit: float() | nil,
          motor_acceleration_limit: float() | nil,
          motor_effort_limit: float() | nil,
          motor_initial_position: float()
        }

  @doc """
  Build a motor profile for an actuator attached to `joint`, applying
  `transmission` (which may be `nil`) to convert joint-space limits into
  motor-space.

  When the joint is `nil` or has no limits, returns a profile with every
  field set to `nil` apart from `motor_initial_position`, which defaults to
  `0.0`.
  """
  @spec from_joint(map() | nil, Transmission.t() | nil) :: t()
  def from_joint(nil, _transmission), do: %__MODULE__{}

  def from_joint(%{limits: nil}, _transmission), do: %__MODULE__{}

  def from_joint(%{limits: limits}, transmission) do
    {motor_lower, motor_upper} = motor_position_limits(limits, transmission)

    %__MODULE__{
      motor_lower: motor_lower,
      motor_upper: motor_upper,
      motor_velocity_limit: motor_rate(Map.get(limits, :velocity), transmission),
      motor_acceleration_limit: motor_rate(Map.get(limits, :acceleration), transmission),
      motor_effort_limit: motor_effort(Map.get(limits, :effort), transmission),
      motor_initial_position: motor_midpoint(motor_lower, motor_upper)
    }
  end

  defp motor_position_limits(%{lower: nil, upper: nil}, _t), do: {nil, nil}

  defp motor_position_limits(%{lower: lower, upper: upper}, nil), do: {lower, upper}

  defp motor_position_limits(%{lower: lower, upper: upper}, transmission) do
    a = if lower, do: Transmission.apply_position(lower, transmission)
    b = if upper, do: Transmission.apply_position(upper, transmission)

    if a && b do
      {min(a, b), max(a, b)}
    else
      {a, b}
    end
  end

  defp motor_rate(nil, _t), do: nil
  defp motor_rate(value, nil), do: value
  defp motor_rate(value, transmission), do: abs(Transmission.apply_rate(value, transmission))

  defp motor_effort(nil, _t), do: nil
  defp motor_effort(value, nil), do: value
  defp motor_effort(value, transmission), do: abs(Transmission.apply_effort(value, transmission))

  defp motor_midpoint(nil, _), do: 0.0
  defp motor_midpoint(_, nil), do: 0.0
  defp motor_midpoint(lower, upper), do: (lower + upper) / 2
end
