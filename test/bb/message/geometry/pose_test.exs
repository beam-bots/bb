# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.PoseTest do
  use ExUnit.Case, async: true

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Pose

  test "creates a pose message" do
    {:ok, msg} = Pose.new(:end_effector, Vec3.new(1, 0, 0.5), Quaternion.identity())

    assert %Message{payload: %Pose{}} = msg
    assert msg.frame_id == :end_effector
  end

  test "validates position is a Vec3" do
    assert {:error, _} =
             Message.new(Pose, :test,
               position: "not a vec3",
               orientation: Quaternion.identity()
             )
  end

  test "validates orientation is a Quaternion" do
    assert {:error, _} =
             Message.new(Pose, :test,
               position: Vec3.zero(),
               orientation: "not a quaternion"
             )
  end
end
