# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.PoseTest do
  use ExUnit.Case, async: true

  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Pose

  test "creates a pose message from Transform" do
    transform = Transform.from_position_quaternion(Vec3.new(1, 0, 0.5), Quaternion.identity())
    {:ok, msg} = Pose.new(:end_effector, transform)

    assert %Message{payload: %Pose{}} = msg
    assert msg.frame_id == :end_effector
  end

  test "creates a pose message from Vec3 and Quaternion" do
    {:ok, msg} = Pose.new(:end_effector, Vec3.new(1, 0, 0.5), Quaternion.identity())

    assert %Message{payload: %Pose{}} = msg
    assert msg.frame_id == :end_effector
  end

  test "extracts position and orientation from pose" do
    {:ok, msg} = Pose.new(:test, Vec3.new(1.0, 2.0, 3.0), Quaternion.identity())
    pose = msg.payload

    position = Pose.position(pose)
    assert_in_delta Vec3.x(position), 1.0, 1.0e-10
    assert_in_delta Vec3.y(position), 2.0, 1.0e-10
    assert_in_delta Vec3.z(position), 3.0, 1.0e-10

    orientation = Pose.orientation(pose)
    assert %Quaternion{} = orientation
  end

  test "validates transform is a Transform" do
    assert {:error, _} =
             Message.new(Pose, :test, transform: "not a transform")
  end
end
