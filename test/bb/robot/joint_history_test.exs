# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.JointHistoryTest do
  use ExUnit.Case, async: true

  alias BB.Dsl.Info
  alias BB.Error.State.JointHistoryDisabled
  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Robot.Runtime
  alias BB.Robot.State

  defmodule DefaultHistoryRobot do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :pan do
          type(:revolute)

          axis do
          end

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(90 degree_per_second))
          end

          link(:head)
        end
      end
    end
  end

  defmodule DisabledHistoryRobot do
    @moduledoc false
    use BB

    settings do
      joint_history false
    end

    topology do
      link :base do
      end
    end
  end

  defmodule CustomHistoryRobot do
    @moduledoc false
    use BB
    import BB.Unit

    settings do
      joint_history do
        keep ~u(30 second)
        interpolation? false
        max_gap ~u(250 millisecond)
      end
    end

    topology do
      link :base do
      end
    end
  end

  defmodule DisabledWithOptionsRobot do
    @moduledoc false
    use BB
    import BB.Unit

    settings do
      joint_history false do
        keep ~u(30 second)
      end
    end

    topology do
      link :base do
      end
    end
  end

  describe "settings" do
    test "a robot that doesn't declare joint_history gets the defaults" do
      assert Info.joint_history(DefaultHistoryRobot) == %{
               keep: 10_000_000_000,
               interpolation?: true,
               max_gap: 1_000_000_000
             }
    end

    test "joint_history false turns it off" do
      assert Info.joint_history(DisabledHistoryRobot) == nil
    end

    test "options are converted to nanoseconds" do
      assert Info.joint_history(CustomHistoryRobot) == %{
               keep: 30_000_000_000,
               interpolation?: false,
               max_gap: 250_000_000
             }
    end

    test "turning it off ignores any other options" do
      assert Info.joint_history(DisabledWithOptionsRobot) == nil
    end
  end

  describe "a running robot" do
    test "records joint state at the message's own time" do
      start_supervised!(DefaultHistoryRobot)
      measured_at = System.monotonic_time(:nanosecond)

      message = %{
        Message.new!(JointState, :pan, names: [:pan], positions: [0.25])
        | monotonic_time: measured_at
      }

      :ok = BB.PubSub.publish(DefaultHistoryRobot, [:sensor, :pan], message)

      # A call queues behind the published message, so the runtime has handled
      # it by the time this returns.
      robot_state = Runtime.get_robot_state(DefaultHistoryRobot)

      assert {:ok, %{pan: 0.25}} = State.get_all_configurations_at(robot_state, measured_at)
    end

    test "keeps no history when it is turned off" do
      start_supervised!(DisabledHistoryRobot)
      robot_state = Runtime.get_robot_state(DisabledHistoryRobot)

      assert {:error, %JointHistoryDisabled{}} =
               State.get_all_configurations_at(robot_state, System.monotonic_time(:nanosecond))
    end
  end
end
