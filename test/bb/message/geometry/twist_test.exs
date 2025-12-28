# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.TwistTest do
  use ExUnit.Case, async: true

  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Twist

  test "creates a twist message" do
    {:ok, msg} = Twist.new(:base_link, Vec3.new(1, 0, 0), Vec3.zero())

    assert %Message{payload: %Twist{}} = msg
    assert msg.frame_id == :base_link
  end
end
