# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.RangeTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Sensor.Range

  test "creates a range message with float value" do
    {:ok, msg} =
      Range.new(:ultrasonic,
        radiation_type: :ultrasound,
        field_of_view: 0.26,
        min_range: 0.02,
        max_range: 4.0,
        range: 1.5
      )

    assert %Message{payload: %Range{}} = msg
    assert msg.payload.radiation_type == :ultrasound
    assert msg.payload.range == 1.5
  end

  test "creates a range message with infinity" do
    {:ok, msg} =
      Range.new(:ultrasonic,
        radiation_type: :infrared,
        field_of_view: 0.1,
        min_range: 0.01,
        max_range: 2.0,
        range: :infinity
      )

    assert msg.payload.range == :infinity
  end
end
