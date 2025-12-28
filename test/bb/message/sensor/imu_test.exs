# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.ImuTest do
  use ExUnit.Case, async: true

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu

  test "creates an IMU message" do
    {:ok, msg} =
      Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0, 0, 9.81)
      )

    assert %Message{payload: %Imu{}} = msg
    assert msg.frame_id == :imu_link
  end
end
