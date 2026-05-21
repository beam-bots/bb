# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.ActuatorTransmissionTest do
  use ExUnit.Case, async: false

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.PubSub

  describe "sim honours the joint's transmission" do
    defmodule SimTxRobot do
      @moduledoc false
      use BB

      topology do
        link :base do
          joint :shoulder do
            type :revolute

            limit do
              lower(~u(-180 degree))
              upper(~u(180 degree))
              effort(~u(10 newton_meter))
              velocity(~u(60 degree_per_second))
              acceleration(~u(120 degree_per_square_second))
            end

            actuator :motor, ServoMotor do
              transmission do
                reduction 50.0
                offset(~u(45 degree))
                reversed? true
              end
            end

            sensor :estimator, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor}

            link :arm
          end
        end
      end
    end

    test "BeginMotion target_position is joint-space, not motor-space" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = SimTxRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(SimTxRobot, [:actuator, :base, :shoulder, :motor])
      :ok = BB.Safety.arm(SimTxRobot)

      joint_target = :math.pi() / 4 + 0.01
      BB.Actuator.set_position!(SimTxRobot, :motor, joint_target)

      assert_receive {:bb, _path,
                      %Message{
                        payload: %BeginMotion{target_position: target}
                      }},
                     1000

      assert_in_delta target, joint_target, 1.0e-9

      Supervisor.stop(pid)
    end
  end
end
