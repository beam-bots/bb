# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.SupervisorTest do
  use ExUnit.Case, async: true
  alias Kinetix.Process, as: KinetixProcess

  defmodule TestGenServer do
    use GenServer

    def init(opts) do
      {:ok, opts}
    end

    def handle_call(:get_state, _from, state) do
      {:reply, state, state}
    end
  end

  describe "basic robot supervision" do
    defmodule BasicRobot do
      @moduledoc false
      use Kinetix

      robot do
        link :base_link do
        end
      end
    end

    test "robot module has start_link/1" do
      assert function_exported?(BasicRobot, :start_link, 1)
    end

    test "robot module has child_spec/1" do
      assert function_exported?(BasicRobot, :child_spec, 1)
    end

    test "child_spec returns valid supervisor spec" do
      spec = BasicRobot.child_spec([])
      assert spec.id == BasicRobot
      assert spec.type == :supervisor
      assert {BasicRobot, :start_link, [[]]} = spec.start
    end

    test "can start the robot" do
      pid = start_supervised!(BasicRobot)
      assert is_pid(pid)
      assert Process.alive?(pid)
    end
  end

  describe "robot with sensors at all levels" do
    defmodule SensorRobot do
      @moduledoc false
      use Kinetix

      robot do
        sensor :camera, Kinetix.SupervisorTest.TestGenServer

        link :base_link do
          sensor :imu, {Kinetix.SupervisorTest.TestGenServer, frequency: 100}

          joint :shoulder do
            type :revolute

            sensor :encoder, Kinetix.SupervisorTest.TestGenServer

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

    test "sensors are registered by name" do
      start_supervised!(SensorRobot)

      camera_pid = KinetixProcess.whereis(SensorRobot, :camera)
      assert is_pid(camera_pid)

      imu_pid = KinetixProcess.whereis(SensorRobot, :imu)
      assert is_pid(imu_pid)

      encoder_pid = KinetixProcess.whereis(SensorRobot, :encoder)
      assert is_pid(encoder_pid)
    end

    test "sensor receives kinetix context in init" do
      start_supervised!(SensorRobot)

      imu_pid = KinetixProcess.whereis(SensorRobot, :imu)
      state = GenServer.call(imu_pid, :get_state)

      assert state[:frequency] == 100
      assert state[:kinetix] == %{robot: SensorRobot, path: [:base_link, :imu]}
    end
  end

  describe "robot with actuators" do
    defmodule ActuatorRobot do
      @moduledoc false
      use Kinetix

      robot do
        link :base_link do
          joint :shoulder do
            type :revolute

            actuator :motor, {Kinetix.SupervisorTest.TestGenServer, pwm_pin: 12}
            actuator :brake, Kinetix.SupervisorTest.TestGenServer

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

    test "actuators are registered by name" do
      start_supervised!(ActuatorRobot)

      motor_pid = KinetixProcess.whereis(ActuatorRobot, :motor)
      assert is_pid(motor_pid)

      brake_pid = KinetixProcess.whereis(ActuatorRobot, :brake)
      assert is_pid(brake_pid)
    end

    test "actuator receives kinetix context and user options in init" do
      start_supervised!(ActuatorRobot)

      motor_pid = KinetixProcess.whereis(ActuatorRobot, :motor)
      state = GenServer.call(motor_pid, :get_state)

      assert state[:pwm_pin] == 12
      assert state[:kinetix] == %{robot: ActuatorRobot, path: [:base_link, :shoulder, :motor]}
    end
  end

  describe "nested robot topology" do
    defmodule NestedRobot do
      @moduledoc false
      use Kinetix

      robot do
        link :base do
          joint :shoulder do
            type :revolute
            actuator :shoulder_motor, Kinetix.SupervisorTest.TestGenServer

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :upper_arm do
              joint :elbow do
                type :revolute
                actuator :elbow_motor, Kinetix.SupervisorTest.TestGenServer

                limit do
                  effort(~u(5 newton_meter))
                  velocity(~u(90 degree_per_second))
                end

                link :forearm do
                  joint :wrist do
                    type :revolute
                    actuator :wrist_motor, Kinetix.SupervisorTest.TestGenServer

                    limit do
                      effort(~u(2 newton_meter))
                      velocity(~u(120 degree_per_second))
                    end

                    link :hand do
                    end
                  end
                end
              end
            end
          end
        end
      end
    end

    test "deeply nested actuators are registered by name" do
      start_supervised!(NestedRobot)

      shoulder_pid = KinetixProcess.whereis(NestedRobot, :shoulder_motor)
      assert is_pid(shoulder_pid)

      elbow_pid = KinetixProcess.whereis(NestedRobot, :elbow_motor)
      assert is_pid(elbow_pid)

      wrist_pid = KinetixProcess.whereis(NestedRobot, :wrist_motor)
      assert is_pid(wrist_pid)
    end
  end

  describe "via tuple" do
    test "via/2 returns correct via tuple" do
      via = KinetixProcess.via(SomeRobot, :motor)

      assert {:via, Registry, {SomeRobot.Registry, :motor}} = via
    end
  end

  describe "process lookup" do
    defmodule LookupRobot do
      @moduledoc false
      use Kinetix

      robot do
        link :base do
          sensor :test_sensor, Kinetix.SupervisorTest.TestGenServer
        end
      end
    end

    test "whereis returns pid for existing process" do
      start_supervised!(LookupRobot)

      sensor_pid = KinetixProcess.whereis(LookupRobot, :test_sensor)
      assert is_pid(sensor_pid)
    end

    test "whereis returns :undefined for non-existent name" do
      start_supervised!(LookupRobot)

      result = KinetixProcess.whereis(LookupRobot, :nonexistent)
      assert result == :undefined
    end
  end
end
