# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.JointStateTest do
  use ExUnit.Case, async: true

  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Geometry.Twist
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Geometry.Wrench
  alias BB.Message.Geometry.Wrench2D
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

  describe "multi-DoF joints" do
    # The whole point of widening this message rather than adding a second one:
    # a mobile base's pose and its mast's angle arrive together, in one instant,
    # so no consumer has to correlate two topics by timestamp.
    test "carry a planar joint alongside a single-DoF joint" do
      configuration = Transform2D.new(12.4, -3.1, 1.57)

      {:ok, msg} =
        JointState.new(:rover,
          names: [:base, :mast],
          positions: [configuration, 0.5],
          velocities: [%Twist2D{vx: 0.3, vy: 0.0, omega: 0.1}, 0.05],
          efforts: [%Wrench2D{fx: 12.0, fy: 0.0, tau: 0.4}, 0.2]
        )

      assert msg.payload.positions == [configuration, 0.5]
      assert [%Twist2D{vx: 0.3}, 0.05] = msg.payload.velocities
      assert [%Wrench2D{fx: 12.0}, 0.2] = msg.payload.efforts
    end

    test "carry a floating joint" do
      pose = Transform.translation(Vec3.new(1.0, 2.0, 3.0))

      {:ok, msg} =
        JointState.new(:drone,
          names: [:base, :gimbal],
          positions: [pose, 0.5],
          velocities: [%Twist{linear: Vec3.zero(), angular: Vec3.zero()}, 0.0],
          efforts: [%Wrench{force: Vec3.zero(), torque: Vec3.zero()}, 0.0]
        )

      assert [^pose, 0.5] = msg.payload.positions
      assert [%Twist{}, +0.0] = msg.payload.velocities
      assert [%Wrench{}, +0.0] = msg.payload.efforts
    end
  end

  describe "validation" do
    # Widening the lists must not mean accepting anything. The message cannot
    # check a value against a *particular* joint's type — that needs the robot,
    # which it does not carry — but it can reject something that is no joint value
    # at all.
    test "rejects a value that is no joint configuration at all" do
      assert {:error, error} =
               JointState.new(:arm, names: [:joint1], positions: ["not a configuration"])

      assert Exception.message(error) =~ "element 0"
    end

    test "names the offending index, since the lists run parallel to names" do
      assert {:error, error} =
               JointState.new(:arm,
                 names: [:a, :b, :c],
                 positions: [0.1, 0.2, :nope]
               )

      assert Exception.message(error) =~ "element 2"
    end

    test "rejects a velocity in the positions list" do
      twist = %Twist2D{vx: 0.0, vy: 0.0, omega: 0.0}

      assert {:error, _} = JointState.new(:arm, names: [:joint1], positions: [twist])
    end

    test "rejects a configuration in the velocities list" do
      assert {:error, _} =
               JointState.new(:arm, names: [:joint1], velocities: [Transform2D.identity()])
    end

    test "rejects a velocity in the efforts list" do
      twist = %Twist2D{vx: 0.0, vy: 0.0, omega: 0.0}

      assert {:error, _} = JointState.new(:arm, names: [:joint1], efforts: [twist])
    end
  end
end
