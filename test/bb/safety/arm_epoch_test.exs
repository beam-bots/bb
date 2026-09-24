# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.ArmEpochTest do
  @moduledoc """
  Tests for arm epoch allocation and invalidation.

  These run synchronously because they interact with the global
  `BB.Safety.Controller` GenServer and need robots in a known state.
  """
  use ExUnit.Case, async: false

  defmodule Robot do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :motor, BB.Test.FailingActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule RobotWithFailingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :failing, {BB.Test.FailingActuator, fail_mode: :error}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule RobotWithSlowActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :slow, {BB.Test.FailingActuator, fail_mode: :slow}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  describe "allocation" do
    test "arming allocates an epoch" do
      start_supervised!(Robot)

      assert :error = BB.Safety.epoch(Robot)

      :ok = BB.Safety.arm(Robot)

      assert {:ok, epoch} = BB.Safety.epoch(Robot)
      assert is_integer(epoch)
    end

    test "a second arming session gets a different epoch" do
      start_supervised!(Robot)

      :ok = BB.Safety.arm(Robot)
      {:ok, first} = BB.Safety.epoch(Robot)

      :ok = BB.Safety.disarm(Robot)
      :ok = BB.Safety.arm(Robot)
      {:ok, second} = BB.Safety.epoch(Robot)

      refute second == first
    end

    test "the epoch is stable for as long as the robot stays armed" do
      start_supervised!(Robot)

      :ok = BB.Safety.arm(Robot)

      assert BB.Safety.epoch(Robot) == BB.Safety.epoch(Robot)
    end

    test "a failed arm allocates nothing" do
      start_supervised!(Robot)

      :ok = BB.Safety.arm(Robot)
      {:ok, epoch} = BB.Safety.epoch(Robot)

      assert {:error, :already_armed} = BB.Safety.arm(Robot)
      assert {:ok, ^epoch} = BB.Safety.epoch(Robot)
    end
  end

  describe "invalidation" do
    test "disarming leaves no epoch" do
      start_supervised!(Robot)

      :ok = BB.Safety.arm(Robot)
      :ok = BB.Safety.disarm(Robot)

      assert :error = BB.Safety.epoch(Robot)
    end

    test "the epoch is gone before the disarm callbacks run" do
      start_supervised!(RobotWithSlowActuator)

      :ok = BB.Safety.arm(RobotWithSlowActuator)
      assert {:ok, _} = BB.Safety.epoch(RobotWithSlowActuator)

      # The slow actuator sleeps for longer than the disarm timeout, so the
      # robot sits in `:disarming` while this task runs.
      task = Task.async(fn -> BB.Safety.disarm(RobotWithSlowActuator) end)

      assert eventually(fn -> BB.Safety.disarming?(RobotWithSlowActuator) end)
      assert :error = BB.Safety.epoch(RobotWithSlowActuator)

      Task.shutdown(task, :brutal_kill)
    end

    test "a failed disarm leaves no epoch either" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert BB.Safety.state(RobotWithFailingActuator) == :error
      assert :error = BB.Safety.epoch(RobotWithFailingActuator)
    end

    test "force_disarm/1 does not resurrect the epoch" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert BB.Safety.state(RobotWithFailingActuator) == :disarmed
      assert :error = BB.Safety.epoch(RobotWithFailingActuator)
    end
  end

  describe "epoch/1" do
    test "an unregistered robot has no epoch" do
      assert :error = BB.Safety.epoch(__MODULE__.NoSuchRobot)
    end

    test "a registered but disarmed robot has no epoch" do
      start_supervised!(Robot)

      assert BB.Safety.state(Robot) == :disarmed
      assert :error = BB.Safety.epoch(Robot)
    end
  end

  defp eventually(fun, remaining \\ 100)
  defp eventually(_fun, 0), do: false

  defp eventually(fun, remaining) do
    if fun.() do
      true
    else
      Process.sleep(10)
      eventually(fun, remaining - 1)
    end
  end
end
