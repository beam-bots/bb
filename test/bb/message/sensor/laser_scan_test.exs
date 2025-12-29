# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.LaserScanTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Sensor.LaserScan

  test "creates a laser scan message" do
    {:ok, msg} =
      LaserScan.new(:laser_frame,
        angle_min: -1.57,
        angle_max: 1.57,
        angle_increment: 0.01,
        time_increment: 0.0001,
        scan_time: 0.1,
        range_min: 0.1,
        range_max: 10.0,
        ranges: [1.0, 1.1, 1.2]
      )

    assert %Message{payload: %LaserScan{}} = msg
    assert msg.frame_id == :laser_frame
    assert msg.payload.intensities == []
  end
end
