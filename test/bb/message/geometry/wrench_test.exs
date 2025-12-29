# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.WrenchTest do
  use ExUnit.Case, async: true

  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Wrench

  test "creates a wrench message" do
    {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0, 0, -10), Vec3.zero())

    assert %Message{payload: %Wrench{}} = msg
    assert msg.frame_id == :end_effector
  end
end
