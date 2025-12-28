# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.JointStateTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Sensor.JointState

  test "creates a joint state message" do
    {:ok, msg} =
      JointState.new(:arm,
        names: [:joint1, :joint2],
        positions: [0.0, 1.57],
        velocities: [0.1, 0.0],
        efforts: [0.5, 0.2]
      )

    assert %Message{payload: %JointState{}} = msg
    assert msg.payload.names == [:joint1, :joint2]
  end

  test "allows empty position/velocity/effort lists" do
    {:ok, msg} = JointState.new(:arm, names: [:joint1])

    assert msg.payload.positions == []
    assert msg.payload.velocities == []
    assert msg.payload.efforts == []
  end
end
