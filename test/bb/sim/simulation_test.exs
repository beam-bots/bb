# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.SimulationTest do
  use ExUnit.Case, async: false

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.PubSub
  alias BB.Robot.Runtime

  describe "simulation mode" do
    defmodule SimModeRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type(:revolute)

            limit do
              lower(~u(-90 degree))
              upper(~u(90 degree))
              velocity(~u(60 degree_per_second))
              effort(~u(10 newton_meter))
            end

            actuator(:motor, ServoMotor)
            sensor(:estimator, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor})

            link :arm do
            end
          end
        end
      end
    end

    setup do
      on_exit(fn ->
        case Process.whereis(SimModeRobot.Registry) do
          nil -> :ok
          _pid -> :ok
        end
      end)

      :ok
    end

    test "robot starts in simulation mode with simulation: :kinematic" do
      {:ok, pid} = SimModeRobot.start_link(simulation: :kinematic)
      assert is_pid(pid)
      assert Runtime.simulation_mode(SimModeRobot) == :kinematic
      Supervisor.stop(pid)
    end

    test "robot starts in hardware mode by default" do
      {:ok, pid} = SimModeRobot.start_link()
      assert is_pid(pid)
      assert Runtime.simulation_mode(SimModeRobot) == nil
      Supervisor.stop(pid)
    end

    test "simulated actuator publishes BeginMotion on position command" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = SimModeRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(SimModeRobot, [:actuator, :base_link, :shoulder, :motor])

      :ok = BB.Safety.arm(SimModeRobot)

      target_position = 0.5
      BB.Actuator.set_position!(SimModeRobot, :motor, target_position)

      assert_receive {:bb, [:actuator, :base_link, :shoulder, :motor],
                      %Message{payload: %BeginMotion{target_position: ^target_position}}},
                     1000

      Supervisor.stop(pid)
    end

    test "simulated actuator clamps position to joint limits" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = SimModeRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(SimModeRobot, [:actuator, :base_link, :shoulder, :motor])

      :ok = BB.Safety.arm(SimModeRobot)

      over_limit = 3.0
      BB.Actuator.set_position!(SimModeRobot, :motor, over_limit)

      upper_limit = :math.pi() / 2

      assert_receive {:bb, [:actuator, :base_link, :shoulder, :motor],
                      %Message{payload: %BeginMotion{target_position: target}}},
                     1000

      assert_in_delta target, upper_limit, 0.001

      Supervisor.stop(pid)
    end

    test "synchronous position command returns acknowledgement" do
      {:ok, pid} = SimModeRobot.start_link(simulation: :kinematic)

      :ok = BB.Safety.arm(SimModeRobot)

      result = BB.Actuator.set_position_sync(SimModeRobot, :motor, 0.5)

      assert result == {:ok, :accepted}

      Supervisor.stop(pid)
    end
  end

  describe "controller simulation options" do
    defmodule CtrlOmitRobot do
      @moduledoc false
      use BB

      controllers do
        controller(:mock_ctrl, {BB.Test.MockController, []}, simulation: :omit)
      end

      topology do
        link :base do
        end
      end
    end

    defmodule CtrlMockRobot do
      @moduledoc false
      use BB

      controllers do
        controller(:mock_ctrl, {BB.Test.MockController, []}, simulation: :mock)
      end

      topology do
        link :base do
        end
      end
    end

    defmodule CtrlStartRobot do
      @moduledoc false
      use BB

      controllers do
        controller(:mock_ctrl, {BB.Test.MockController, []}, simulation: :start)
      end

      topology do
        link :base do
        end
      end
    end

    test "controller with simulation: :omit is not started in simulation mode" do
      {:ok, pid} = CtrlOmitRobot.start_link(simulation: :kinematic)

      assert BB.Process.whereis(CtrlOmitRobot, :mock_ctrl) == :undefined

      Supervisor.stop(pid)
    end

    test "controller with simulation: :omit is started in hardware mode" do
      {:ok, pid} = CtrlOmitRobot.start_link()

      refute BB.Process.whereis(CtrlOmitRobot, :mock_ctrl) == :undefined

      Supervisor.stop(pid)
    end

    test "controller with simulation: :mock starts mock controller" do
      {:ok, pid} = CtrlMockRobot.start_link(simulation: :kinematic)

      refute BB.Process.whereis(CtrlMockRobot, :mock_ctrl) == :undefined

      Supervisor.stop(pid)
    end

    test "controller with simulation: :start starts real controller" do
      {:ok, pid} = CtrlStartRobot.start_link(simulation: :kinematic)

      refute BB.Process.whereis(CtrlStartRobot, :mock_ctrl) == :undefined

      Supervisor.stop(pid)
    end
  end

  describe "automatic position estimator" do
    alias BB.Message.Sensor.JointState

    defmodule AutoEstimatorRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type(:revolute)

            limit do
              lower(~u(-90 degree))
              upper(~u(90 degree))
              velocity(~u(60 degree_per_second))
              effort(~u(10 newton_meter))
            end

            actuator(:motor, ServoMotor)
            # No sensor - should be auto-added in simulation mode

            link :arm do
            end
          end
        end
      end
    end

    defmodule ManualEstimatorRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type(:revolute)

            limit do
              lower(~u(-90 degree))
              upper(~u(90 degree))
              velocity(~u(60 degree_per_second))
              effort(~u(10 newton_meter))
            end

            actuator(:motor, ServoMotor)
            sensor(:my_estimator, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor})

            link :arm do
            end
          end
        end
      end
    end

    test "automatically adds OpenLoopPositionEstimator in simulation mode when no sensor exists" do
      {:ok, pid} = AutoEstimatorRobot.start_link(simulation: :kinematic)

      # The auto-generated sensor should be named after the actuator
      refute BB.Process.whereis(AutoEstimatorRobot, :motor_position_estimator) == :undefined

      Supervisor.stop(pid)
    end

    test "does not add auto estimator when one already exists for the actuator" do
      {:ok, pid} = ManualEstimatorRobot.start_link(simulation: :kinematic)

      # Manual sensor exists
      refute BB.Process.whereis(ManualEstimatorRobot, :my_estimator) == :undefined

      # Auto sensor should not be created
      assert BB.Process.whereis(ManualEstimatorRobot, :motor_position_estimator) == :undefined

      Supervisor.stop(pid)
    end

    test "auto estimator publishes JointState when actuator receives position command" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = AutoEstimatorRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(AutoEstimatorRobot, [:sensor])

      :ok = BB.Safety.arm(AutoEstimatorRobot)

      target_position = 0.5
      BB.Actuator.set_position!(AutoEstimatorRobot, :motor, target_position)

      # Should receive JointState from the auto-added estimator
      assert_receive {:bb, [:sensor, :base_link, :shoulder, :motor_position_estimator],
                      %Message{payload: %JointState{positions: [position]}}},
                     1000

      # Position should be at or approaching target
      assert is_float(position)

      Supervisor.stop(pid)
    end

    test "does not add auto estimator in hardware mode" do
      {:ok, pid} = AutoEstimatorRobot.start_link()

      # No auto sensor in hardware mode
      assert BB.Process.whereis(AutoEstimatorRobot, :motor_position_estimator) == :undefined

      Supervisor.stop(pid)
    end
  end

  describe "bridge simulation options" do
    defmodule BridgeOmitRobot do
      @moduledoc false
      use BB

      parameters do
        bridge(:mock_bridge, {BB.Test.MockBridge, []}, simulation: :omit)
      end

      topology do
        link :base do
        end
      end
    end

    defmodule BridgeMockRobot do
      @moduledoc false
      use BB

      parameters do
        bridge(:mock_bridge, {BB.Test.MockBridge, []}, simulation: :mock)
      end

      topology do
        link :base do
        end
      end
    end

    defmodule BridgeStartRobot do
      @moduledoc false
      use BB

      parameters do
        bridge(:mock_bridge, {BB.Test.MockBridge, []}, simulation: :start)
      end

      topology do
        link :base do
        end
      end
    end

    test "bridge with simulation: :omit is not started in simulation mode" do
      {:ok, pid} = BridgeOmitRobot.start_link(simulation: :kinematic)

      assert BB.Process.whereis(BridgeOmitRobot, :mock_bridge) == :undefined

      Supervisor.stop(pid)
    end

    test "bridge with simulation: :omit is started in hardware mode" do
      {:ok, pid} = BridgeOmitRobot.start_link()

      refute BB.Process.whereis(BridgeOmitRobot, :mock_bridge) == :undefined

      Supervisor.stop(pid)
    end

    test "bridge with simulation: :mock starts mock bridge" do
      {:ok, pid} = BridgeMockRobot.start_link(simulation: :kinematic)

      refute BB.Process.whereis(BridgeMockRobot, :mock_bridge) == :undefined

      Supervisor.stop(pid)
    end

    test "bridge with simulation: :start starts real bridge" do
      {:ok, pid} = BridgeStartRobot.start_link(simulation: :kinematic)

      refute BB.Process.whereis(BridgeStartRobot, :mock_bridge) == :undefined

      Supervisor.stop(pid)
    end
  end
end
