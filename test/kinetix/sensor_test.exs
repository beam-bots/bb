# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.SensorTest do
  use ExUnit.Case, async: true
  alias Kinetix.Dsl.{Info, Link, Sensor}

  describe "robot-level sensor" do
    defmodule RobotLevelSensorRobot do
      @moduledoc false
      use Kinetix

      sensors do
        sensor :camera, MySensor
      end

      topology do
        link :base_link do
        end
      end
    end

    test "sensor defined at robot level" do
      sensors = Info.sensors(RobotLevelSensorRobot)
      sensor = Enum.find(sensors, &is_struct(&1, Sensor))
      assert sensor.name == :camera
      assert sensor.child_spec == MySensor
    end
  end

  describe "link-level sensor" do
    defmodule LinkSensorRobot do
      @moduledoc false
      use Kinetix

      topology do
        link :base_link do
          sensor :imu, {MySensor, frequency: 100}
        end
      end
    end

    test "sensor attached to link with module and args" do
      [link] = Info.topology(LinkSensorRobot)
      [sensor] = link.sensors
      assert is_struct(sensor, Sensor)
      assert sensor.name == :imu
      assert sensor.child_spec == {MySensor, [frequency: 100]}
    end
  end

  describe "multiple sensors" do
    defmodule MultipleSensorsRobot do
      @moduledoc false
      use Kinetix

      sensors do
        sensor :camera, CameraSensor
      end

      topology do
        link :base_link do
          sensor :imu, ImuSensor
          sensor :gps, {GpsSensor, port: "/dev/ttyUSB0"}
        end
      end
    end

    test "multiple sensors on a single link" do
      entities = Info.topology(MultipleSensorsRobot)
      [link] = Enum.filter(entities, &is_struct(&1, Link))
      assert length(link.sensors) == 2
    end

    test "sensors at both link and robot level" do
      robot_sensors = Info.sensors(MultipleSensorsRobot)
      assert length(robot_sensors) == 1
      assert hd(robot_sensors).name == :camera
    end
  end

  describe "nested link sensors" do
    defmodule NestedLinkSensorRobot do
      @moduledoc false
      use Kinetix

      topology do
        link :base_link do
          sensor :base_sensor, BaseSensor

          joint :joint1 do
            type :fixed

            link :child_link do
              sensor :child_sensor, ChildSensor
            end
          end
        end
      end
    end

    test "sensors in nested links" do
      [base_link] = Info.topology(NestedLinkSensorRobot)
      assert length(base_link.sensors) == 1
      assert hd(base_link.sensors).name == :base_sensor

      [joint] = base_link.joints
      child_link = joint.link
      assert length(child_link.sensors) == 1
      assert hd(child_link.sensors).name == :child_sensor
    end
  end

  describe "joint-level sensor" do
    defmodule JointSensorRobot do
      @moduledoc false
      use Kinetix

      topology do
        link :base_link do
          joint :shoulder do
            type :revolute

            sensor :encoder, {Encoder, bus: :i2c1}

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :arm do
            end
          end
        end
      end
    end

    test "sensor attached to joint" do
      [link] = Info.topology(JointSensorRobot)
      [joint] = link.joints
      [sensor] = joint.sensors
      assert is_struct(sensor, Sensor)
      assert sensor.name == :encoder
      assert sensor.child_spec == {Encoder, [bus: :i2c1]}
    end
  end

  describe "sensors on links, joints, and robot" do
    defmodule MixedSensorsRobot do
      @moduledoc false
      use Kinetix

      sensors do
        sensor :robot_sensor, RobotSensor
      end

      topology do
        link :base_link do
          sensor :link_sensor, LinkSensor

          joint :shoulder do
            type :revolute

            sensor :joint_sensor, JointSensor

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :arm do
            end
          end
        end
      end
    end

    test "sensors at all levels" do
      entities = Info.topology(MixedSensorsRobot)
      [link] = Enum.filter(entities, &is_struct(&1, Link))
      [robot_sensor] = Info.sensors(MixedSensorsRobot)

      assert robot_sensor.name == :robot_sensor
      assert length(link.sensors) == 1
      assert hd(link.sensors).name == :link_sensor

      [joint] = link.joints
      assert length(joint.sensors) == 1
      assert hd(joint.sensors).name == :joint_sensor
    end
  end
end
