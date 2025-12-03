# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.RobotTransformerTest do
  use ExUnit.Case, async: true

  describe "name uniqueness validation" do
    test "rejects duplicate link names" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:duplicate/, fn ->
        defmodule DuplicateLinkNames do
          use Kinetix

          topology do
            link :duplicate do
              joint :joint1 do
                type :fixed
                link :duplicate
              end
            end
          end
        end
      end
    end

    test "rejects link and joint with same name" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:shared_name/, fn ->
        defmodule LinkJointSameName do
          use Kinetix

          topology do
            link :base do
              joint :shared_name do
                type :fixed
                link :shared_name
              end
            end
          end
        end
      end
    end

    test "rejects duplicate sensor names across links" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:my_sensor/, fn ->
        defmodule DuplicateSensorNames do
          use Kinetix

          topology do
            link :base do
              sensor :my_sensor, MySensor

              joint :joint1 do
                type :fixed

                link :child do
                  sensor :my_sensor, MySensor
                end
              end
            end
          end
        end
      end
    end

    test "rejects sensor with same name as link" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:base/, fn ->
        defmodule SensorLinkSameName do
          use Kinetix

          topology do
            link :base do
              sensor :base, MySensor
            end
          end
        end
      end
    end

    test "rejects duplicate actuator names" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:motor/, fn ->
        defmodule DuplicateActuatorNames do
          use Kinetix

          topology do
            link :base do
              joint :j1 do
                type :revolute

                limit do
                  effort(~u(10 newton_meter))
                  velocity(~u(1 degree_per_second))
                end

                actuator :motor, MyMotor

                link :link1 do
                  joint :j2 do
                    type :revolute

                    limit do
                      effort(~u(10 newton_meter))
                      velocity(~u(1 degree_per_second))
                    end

                    actuator :motor, MyMotor

                    link :link2
                  end
                end
              end
            end
          end
        end
      end
    end

    test "rejects actuator with same name as joint" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:shoulder/, fn ->
        defmodule ActuatorJointSameName do
          use Kinetix

          topology do
            link :base do
              joint :shoulder do
                type :revolute

                limit do
                  effort(~u(10 newton_meter))
                  velocity(~u(1 degree_per_second))
                end

                actuator :shoulder, MyMotor
                link :arm
              end
            end
          end
        end
      end
    end

    test "rejects robot-level sensor with same name as link" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:base/, fn ->
        defmodule RobotSensorLinkSameName do
          use Kinetix

          sensors do
            sensor :base, MySensor
          end

          topology do
            link :base
          end
        end
      end
    end

    test "accepts unique names across all entities" do
      defmodule UniqueNamesRobot do
        use Kinetix

        sensors do
          sensor :robot_sensor, MySensor
        end

        topology do
          link :base do
            sensor :base_sensor, MySensor

            joint :shoulder do
              type :revolute

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(1 degree_per_second))
              end

              sensor :joint_sensor, MySensor
              actuator :shoulder_motor, MyMotor

              link :arm do
                sensor :arm_sensor, MySensor
              end
            end
          end
        end
      end

      robot = UniqueNamesRobot.robot()
      assert robot.name == UniqueNamesRobot
      assert Map.has_key?(robot.links, :base)
      assert Map.has_key?(robot.links, :arm)
      assert Map.has_key?(robot.joints, :shoulder)
    end
  end
end
