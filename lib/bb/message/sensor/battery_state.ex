# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.BatteryState do
  @moduledoc """
  Battery state information.

  ## Fields

  - `voltage` - Voltage in Volts
  - `current` - Current in Amperes (negative when discharging)
  - `charge` - Charge in Ampere-hours (0 if not measured)
  - `capacity` - Capacity in Ampere-hours (full charge, 0 if not measured)
  - `percentage` - Charge percentage (0.0 to 1.0, or nil if not measured)
  - `power_supply_status` - Status of the power supply
  - `power_supply_health` - Health of the power supply
  - `present` - Whether battery is present

  ## Power Supply Status

  - `:unknown` - Cannot determine status
  - `:charging` - Battery is charging
  - `:discharging` - Battery is discharging
  - `:not_charging` - Not charging (full or error)
  - `:full` - Battery is full

  ## Power Supply Health

  - `:unknown` - Cannot determine health
  - `:good` - Battery is healthy
  - `:overheat` - Battery is overheating
  - `:dead` - Battery is dead
  - `:overvoltage` - Voltage too high
  - `:cold` - Battery is too cold

  ## Examples

      alias BB.Message.Sensor.BatteryState

      {:ok, msg} = BatteryState.new(:battery,
        voltage: 12.6,
        current: -0.5,
        percentage: 0.85,
        power_supply_status: :discharging,
        power_supply_health: :good,
        present: true
      )
  """

  @behaviour BB.Message

  defstruct [
    :voltage,
    :current,
    :charge,
    :capacity,
    :percentage,
    :power_supply_status,
    :power_supply_health,
    :present
  ]

  @type power_supply_status :: :unknown | :charging | :discharging | :not_charging | :full
  @type power_supply_health :: :unknown | :good | :overheat | :dead | :overvoltage | :cold

  @type t :: %__MODULE__{
          voltage: float(),
          current: float(),
          charge: float(),
          capacity: float(),
          percentage: float() | nil,
          power_supply_status: power_supply_status(),
          power_supply_health: power_supply_health(),
          present: boolean()
        }

  @schema Spark.Options.new!(
            voltage: [type: :float, required: true, doc: "Voltage in Volts"],
            current: [type: :float, default: 0.0, doc: "Current in Amperes"],
            charge: [type: :float, default: 0.0, doc: "Charge in Ampere-hours"],
            capacity: [type: :float, default: 0.0, doc: "Capacity in Ampere-hours"],
            percentage: [
              type: {:or, [:float, {:literal, nil}]},
              default: nil,
              doc: "Charge percentage (0.0 to 1.0)"
            ],
            power_supply_status: [
              type: {:in, [:unknown, :charging, :discharging, :not_charging, :full]},
              default: :unknown,
              doc: "Power supply status"
            ],
            power_supply_health: [
              type: {:in, [:unknown, :good, :overheat, :dead, :overvoltage, :cold]},
              default: :unknown,
              doc: "Power supply health"
            ],
            present: [type: :boolean, default: true, doc: "Whether battery is present"]
          )

  @impl BB.Message
  def schema, do: @schema

  defimpl BB.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new BatteryState message.

  Returns `{:ok, %BB.Message{}}` with the battery state as payload.

  ## Examples

      {:ok, msg} = BatteryState.new(:battery,
        voltage: 12.6,
        power_supply_status: :discharging,
        present: true
      )
  """
  @spec new(atom(), keyword()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    BB.Message.new(__MODULE__, frame_id, attrs)
  end
end
