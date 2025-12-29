# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.BatteryStateTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Sensor.BatteryState

  test "creates a battery state message" do
    {:ok, msg} =
      BatteryState.new(:battery,
        voltage: 12.6,
        current: -0.5,
        percentage: 0.85,
        power_supply_status: :discharging,
        power_supply_health: :good
      )

    assert %Message{payload: %BatteryState{}} = msg
    assert msg.payload.power_supply_status == :discharging
    assert msg.payload.present == true
  end

  test "uses defaults for optional fields" do
    {:ok, msg} = BatteryState.new(:battery, voltage: 12.0)

    assert msg.payload.current == 0.0
    assert msg.payload.percentage == nil
    assert msg.payload.power_supply_status == :unknown
  end
end
