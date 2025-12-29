# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.MessageTest do
  use ExUnit.Case, async: true

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Pose

  describe "Message envelope" do
    test "new/3 creates a message with timestamp, frame_id, and payload" do
      {:ok, msg} = Pose.new(:base_link, Vec3.new(1, 2, 3), Quaternion.identity())

      assert %Message{} = msg
      assert is_integer(msg.timestamp)
      assert msg.frame_id == :base_link
      assert %Pose{} = msg.payload
    end

    test "new!/3 raises on validation error" do
      assert_raise Spark.Options.ValidationError, fn ->
        Message.new!(Pose, :base_link, position: "invalid")
      end
    end

    test "schema/1 returns the payload schema" do
      {:ok, msg} = Pose.new(:test, Vec3.zero(), Quaternion.identity())
      schema = Message.schema(msg)
      assert %Spark.Options{} = schema
    end
  end
end
