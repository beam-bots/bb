# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.PowerState do
  @moduledoc """
  Instantaneous electrical state of a measured circuit, bus, or device.

  Published by power-monitor chips (e.g. INA219, INA226) and any device
  that exposes voltage and current as part of its normal telemetry (e.g.
  Robotis or Feetech servos reporting their own draw).

  This is a raw electrical snapshot. Battery-aware concepts like
  state-of-charge, percentage, or health belong in
  `BB.Message.Sensor.BatteryState` and are the job of a downstream
  consumer that subscribes to `PowerState` and maintains charge state.

  ## Fields

  - `voltage` - Bus voltage in Volts
  - `current` - Current in Amperes (signed; sign convention is producer-defined,
    typically positive = flowing in the measured direction)
  - `power` - Power in Watts, or `nil` if the producer doesn't measure it
  - `shunt_voltage` - Shunt voltage in Volts, or `nil` if not exposed (diagnostic)

  ## Examples

      alias BB.Message.Sensor.PowerState

      {:ok, msg} = PowerState.new(:battery_bus,
        voltage: 12.4,
        current: 0.85,
        power: 10.54
      )
  """

  defstruct [:voltage, :current, :power, :shunt_voltage]

  use BB.Message,
    schema: [
      voltage: [type: :float, required: true, doc: "Bus voltage in Volts"],
      current: [type: :float, required: true, doc: "Current in Amperes (signed)"],
      power: [
        type: {:or, [:float, {:literal, nil}]},
        default: nil,
        doc: "Power in Watts (nil if not measured)"
      ],
      shunt_voltage: [
        type: {:or, [:float, {:literal, nil}]},
        default: nil,
        doc: "Shunt voltage in Volts (diagnostic; nil if not exposed)"
      ]
    ]

  @type t :: %__MODULE__{
          voltage: float(),
          current: float(),
          power: float() | nil,
          shunt_voltage: float() | nil
        }
end
