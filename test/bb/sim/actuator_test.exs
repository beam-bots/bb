# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sim.ActuatorTest do
  use ExUnit.Case, async: false

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.PubSub

  describe "trapezoidal profile" do
    defmodule TrapRobot do
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

            actuator :motor, ServoMotor
            sensor :estimator, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor}

            link :arm
          end
        end
      end
    end

    test "BeginMotion carries acceleration and peak_velocity when limits include acceleration" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = TrapRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(TrapRobot, [:actuator, :base, :shoulder, :motor])
      :ok = BB.Safety.arm(TrapRobot)

      # Move from 0 to 1.0 rad. v=60°/s≈1.047 rad/s, a=120°/s²≈2.094 rad/s².
      # 2 * d_accel = 2 * 0.5 * a * (v/a)² = v²/a ≈ 0.524 rad.
      # 1.0 > 0.524, so trapezoid; peak_velocity should equal velocity limit.
      BB.Actuator.set_position!(TrapRobot, :motor, 1.0)

      assert_receive {:bb, _path,
                      %Message{
                        payload: %BeginMotion{
                          acceleration: a,
                          peak_velocity: v
                        }
                      }},
                     1000

      assert_in_delta a, :math.pi() * 120 / 180, 0.001
      assert_in_delta v, :math.pi() * 60 / 180, 0.001

      Supervisor.stop(pid)
    end

    test "BeginMotion uses triangular profile for short moves" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = TrapRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(TrapRobot, [:actuator, :base, :shoulder, :motor])
      :ok = BB.Safety.arm(TrapRobot)

      # Move 0.1 rad — below the cruise threshold of ~0.524 rad. Should be
      # triangular: peak_velocity < velocity limit, peak = sqrt(d * a).
      BB.Actuator.set_position!(TrapRobot, :motor, 0.1)

      assert_receive {:bb, _path,
                      %Message{
                        payload: %BeginMotion{
                          acceleration: a,
                          peak_velocity: v
                        }
                      }},
                     1000

      assert_in_delta a, :math.pi() * 120 / 180, 0.001
      expected_peak = :math.sqrt(0.1 * a)
      assert_in_delta v, expected_peak, 0.001
      assert v < :math.pi() * 60 / 180
      Supervisor.stop(pid)
    end
  end

  describe "rectangular fallback when acceleration is omitted" do
    defmodule RectRobot do
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
            end

            actuator :motor, ServoMotor
            sensor :estimator, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor}

            link :arm
          end
        end
      end
    end

    test "BeginMotion has nil acceleration/peak_velocity when no acceleration limit" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = RectRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(RectRobot, [:actuator, :base, :shoulder, :motor])
      :ok = BB.Safety.arm(RectRobot)

      BB.Actuator.set_position!(RectRobot, :motor, 1.0)

      assert_receive {:bb, _path,
                      %Message{
                        payload: %BeginMotion{acceleration: nil, peak_velocity: nil}
                      }},
                     1000

      Supervisor.stop(pid)
    end
  end

  describe "actual current position under rapid commanding (Bug A)" do
    defmodule RapidRobot do
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
              velocity(~u(10 degree_per_second))
            end

            actuator :motor, ServoMotor
            sensor :estimator, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor}

            link :arm
          end
        end
      end
    end

    test "second command uses interpolated current position as initial_position" do
      Process.flag(:trap_exit, true)
      {:ok, pid} = RapidRobot.start_link(simulation: :kinematic)

      PubSub.subscribe(RapidRobot, [:actuator, :base, :shoulder, :motor])
      :ok = BB.Safety.arm(RapidRobot)

      # First move: 0 → 1.0 rad. Velocity = 10°/s ≈ 0.175 rad/s, so travel
      # time ≈ 5.7s. We deliberately interrupt long before arrival.
      BB.Actuator.set_position!(RapidRobot, :motor, 1.0)

      assert_receive {:bb, _path, %Message{payload: %BeginMotion{}}}, 1000

      Process.sleep(50)
      # Second command: should NOT report initial_position=1.0 (the previous
      # commanded target); it should report a position partway between 0 and
      # 1.0 reflecting elapsed interpolation.
      BB.Actuator.set_position!(RapidRobot, :motor, 0.5)

      assert_receive {:bb, _path,
                      %Message{
                        payload: %BeginMotion{initial_position: actual_initial}
                      }},
                     1000

      assert actual_initial > 0.0,
             "Expected the actuator to have moved away from 0, got #{actual_initial}"

      assert actual_initial < 1.0,
             "Expected the actuator NOT to have reached the previous target (1.0), got #{actual_initial}"

      Supervisor.stop(pid)
    end
  end
end
