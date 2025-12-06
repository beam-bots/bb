# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.MessageTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Geometry.{Accel, Pose, Transform, Twist, Wrench}
  alias BB.Message.{Quaternion, Vec3}
  alias BB.Message.Sensor.{BatteryState, Image, Imu, JointState, LaserScan, Range}

  describe "Vec3" do
    test "new/3 creates a vec3 tuple with float values" do
      assert {:vec3, 1.0, 2.0, 3.0} = Vec3.new(1, 2, 3)
    end

    test "zero/0 returns origin" do
      assert Vec3.zero() == {:vec3, 0.0, 0.0, 0.0}
    end

    test "unit vectors" do
      assert Vec3.unit_x() == {:vec3, 1.0, 0.0, 0.0}
      assert Vec3.unit_y() == {:vec3, 0.0, 1.0, 0.0}
      assert Vec3.unit_z() == {:vec3, 0.0, 0.0, 1.0}
    end

    test "accessors" do
      v = Vec3.new(1, 2, 3)
      assert Vec3.x(v) == 1.0
      assert Vec3.y(v) == 2.0
      assert Vec3.z(v) == 3.0
    end

    test "to_list/1 and from_list/1" do
      v = Vec3.new(1, 2, 3)
      assert Vec3.to_list(v) == [1.0, 2.0, 3.0]
      assert Vec3.from_list([1, 2, 3]) == {:vec3, 1.0, 2.0, 3.0}
    end
  end

  describe "Quaternion" do
    test "new/4 creates a quaternion tuple with float values" do
      assert Quaternion.new(0, 0, 0, 1) == {:quaternion, 0.0, 0.0, 0.0, 1.0}
    end

    test "identity/0 returns identity quaternion" do
      assert Quaternion.identity() == {:quaternion, 0.0, 0.0, 0.0, 1.0}
    end

    test "accessors" do
      q = Quaternion.new(1, 2, 3, 4)
      assert Quaternion.x(q) == 1.0
      assert Quaternion.y(q) == 2.0
      assert Quaternion.z(q) == 3.0
      assert Quaternion.w(q) == 4.0
    end

    test "to_list/1 and from_list/1" do
      q = Quaternion.new(1, 2, 3, 4)
      assert Quaternion.to_list(q) == [1.0, 2.0, 3.0, 4.0]
      assert Quaternion.from_list([1, 2, 3, 4]) == {:quaternion, 1.0, 2.0, 3.0, 4.0}
    end
  end

  describe "Message envelope" do
    test "new/3 creates a message with timestamp, frame_id, and payload" do
      {:ok, msg} =
        Pose.new(
          :base_link,
          Vec3.new(1, 2, 3),
          Quaternion.identity()
        )

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

  describe "Pose" do
    test "creates a pose message" do
      {:ok, msg} = Pose.new(:end_effector, Vec3.new(1, 0, 0.5), Quaternion.identity())

      assert msg.frame_id == :end_effector
      assert msg.payload.position == {:vec3, 1.0, 0.0, 0.5}
      assert msg.payload.orientation == {:quaternion, 0.0, 0.0, 0.0, 1.0}
    end

    test "validates position is a vec3" do
      assert {:error, _} =
               Message.new(Pose, :test,
                 position: "not a vec3",
                 orientation: Quaternion.identity()
               )
    end

    test "validates orientation is a quaternion" do
      assert {:error, _} =
               Message.new(Pose, :test,
                 position: Vec3.zero(),
                 orientation: "not a quaternion"
               )
    end
  end

  describe "Transform" do
    test "creates a transform message" do
      {:ok, msg} = Transform.new(:base_link, Vec3.new(0, 0, 1), Quaternion.identity())

      assert msg.payload.translation == {:vec3, 0.0, 0.0, 1.0}
      assert msg.payload.rotation == {:quaternion, 0.0, 0.0, 0.0, 1.0}
    end
  end

  describe "Twist" do
    test "creates a twist message" do
      {:ok, msg} = Twist.new(:base_link, Vec3.new(1, 0, 0), Vec3.zero())

      assert msg.payload.linear == {:vec3, 1.0, 0.0, 0.0}
      assert msg.payload.angular == {:vec3, 0.0, 0.0, 0.0}
    end
  end

  describe "Accel" do
    test "creates an acceleration message" do
      {:ok, msg} = Accel.new(:base_link, Vec3.new(0, 0, 9.81), Vec3.zero())

      assert msg.payload.linear == {:vec3, 0.0, 0.0, 9.81}
      assert msg.payload.angular == {:vec3, 0.0, 0.0, 0.0}
    end
  end

  describe "Wrench" do
    test "creates a wrench message" do
      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0, 0, -10), Vec3.zero())

      assert msg.payload.force == {:vec3, 0.0, 0.0, -10.0}
      assert msg.payload.torque == {:vec3, 0.0, 0.0, 0.0}
    end
  end

  describe "JointState" do
    test "creates a joint state message" do
      {:ok, msg} =
        JointState.new(:arm,
          names: [:joint1, :joint2],
          positions: [0.0, 1.57],
          velocities: [0.1, 0.0],
          efforts: [0.5, 0.2]
        )

      assert msg.payload.names == [:joint1, :joint2]
      assert msg.payload.positions == [0.0, 1.57]
      assert msg.payload.velocities == [0.1, 0.0]
      assert msg.payload.efforts == [0.5, 0.2]
    end

    test "allows empty position/velocity/effort lists" do
      {:ok, msg} = JointState.new(:arm, names: [:joint1])

      assert msg.payload.positions == []
      assert msg.payload.velocities == []
      assert msg.payload.efforts == []
    end
  end

  describe "Imu" do
    test "creates an IMU message" do
      {:ok, msg} =
        Imu.new(:imu_link,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.new(0, 0, 9.81)
        )

      assert msg.payload.orientation == {:quaternion, 0.0, 0.0, 0.0, 1.0}
      assert msg.payload.angular_velocity == {:vec3, 0.0, 0.0, 0.0}
      assert msg.payload.linear_acceleration == {:vec3, 0.0, 0.0, 9.81}
    end
  end

  describe "LaserScan" do
    test "creates a laser scan message" do
      {:ok, msg} =
        LaserScan.new(:laser_frame,
          angle_min: -1.57,
          angle_max: 1.57,
          angle_increment: 0.01,
          time_increment: 0.0001,
          scan_time: 0.1,
          range_min: 0.1,
          range_max: 10.0,
          ranges: [1.0, 1.1, 1.2]
        )

      assert msg.payload.angle_min == -1.57
      assert msg.payload.angle_max == 1.57
      assert msg.payload.ranges == [1.0, 1.1, 1.2]
      assert msg.payload.intensities == []
    end
  end

  describe "Range" do
    test "creates a range message with float value" do
      {:ok, msg} =
        Range.new(:ultrasonic,
          radiation_type: :ultrasound,
          field_of_view: 0.26,
          min_range: 0.02,
          max_range: 4.0,
          range: 1.5
        )

      assert msg.payload.radiation_type == :ultrasound
      assert msg.payload.range == 1.5
    end

    test "creates a range message with infinity" do
      {:ok, msg} =
        Range.new(:ultrasonic,
          radiation_type: :infrared,
          field_of_view: 0.1,
          min_range: 0.01,
          max_range: 2.0,
          range: :infinity
        )

      assert msg.payload.range == :infinity
    end
  end

  describe "BatteryState" do
    test "creates a battery state message" do
      {:ok, msg} =
        BatteryState.new(:battery,
          voltage: 12.6,
          current: -0.5,
          percentage: 0.85,
          power_supply_status: :discharging,
          power_supply_health: :good
        )

      assert msg.payload.voltage == 12.6
      assert msg.payload.current == -0.5
      assert msg.payload.percentage == 0.85
      assert msg.payload.power_supply_status == :discharging
      assert msg.payload.power_supply_health == :good
      assert msg.payload.present == true
    end

    test "uses defaults for optional fields" do
      {:ok, msg} = BatteryState.new(:battery, voltage: 12.0)

      assert msg.payload.current == 0.0
      assert msg.payload.charge == 0.0
      assert msg.payload.capacity == 0.0
      assert msg.payload.percentage == nil
      assert msg.payload.power_supply_status == :unknown
      assert msg.payload.power_supply_health == :unknown
    end
  end

  describe "Image" do
    test "creates an image message" do
      data = <<0, 0, 0, 255, 255, 255>>

      {:ok, msg} =
        Image.new(:camera,
          height: 1,
          width: 2,
          encoding: :rgb8,
          step: 6,
          data: data
        )

      assert msg.payload.height == 1
      assert msg.payload.width == 2
      assert msg.payload.encoding == :rgb8
      assert msg.payload.is_bigendian == false
      assert msg.payload.step == 6
      assert msg.payload.data == data
    end
  end
end
