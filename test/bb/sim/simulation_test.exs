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
