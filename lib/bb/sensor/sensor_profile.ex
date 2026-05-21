# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.SensorProfile do
  @moduledoc """
  Resolved transmission and joint reference for a joint-attached sensor.

  Built by `BB.Sensor.Server` from a sensor's transmission and joint at
  init, then injected into the sensor callback module's resolved options as
  `:sensor_profile`. Drivers read the resolved transmission from this
  struct instead of looking up `BB.Robot.sensors` themselves.

  When the sensor is attached at the robot level or to a link (not a
  joint), `joint_name` is `nil` and `transmission` is `nil`.
  """

  alias BB.Transmission

  defstruct [
    :joint_name,
    :transmission
  ]

  @type t :: %__MODULE__{
          joint_name: atom() | nil,
          transmission: Transmission.t() | nil
        }
end
