# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.AccelTest do
  use ExUnit.Case, async: true

  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Accel

  test "creates an acceleration message" do
    {:ok, msg} = Accel.new(:base_link, Vec3.new(0, 0, 9.81), Vec3.zero())

    assert %Message{payload: %Accel{}} = msg
    assert msg.frame_id == :base_link
  end
end
