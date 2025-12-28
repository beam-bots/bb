# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.TransformTest do
  use ExUnit.Case, async: true

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Transform

  test "creates a transform message" do
    {:ok, msg} = Transform.new(:base_link, Vec3.new(0, 0, 1), Quaternion.identity())

    assert %Message{payload: %Transform{}} = msg
    assert msg.frame_id == :base_link
  end
end
