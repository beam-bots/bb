# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.ControllerTest do
  @moduledoc """
  Tests for BB.Safety.Controller error state handling and force_disarm/1.
  """
  use ExUnit.Case, async: true

  alias BB.StateMachine.Transition

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

  defmodule RobotWithRaisingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :failing, {BB.Test.FailingActuator, fail_mode: :raise}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule RobotWithThrowingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :failing, {BB.Test.FailingActuator, fail_mode: :throw}

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

  describe "error state on disarm callback failure" do
    test "disarm transitions to error state when callback returns error" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert BB.Safety.state(RobotWithFailingActuator) == :armed

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert BB.Safety.state(RobotWithFailingActuator) == :error
      assert BB.Safety.in_error?(RobotWithFailingActuator) == true
      assert length(failures) == 1
      assert {[:base, :joint1, :failing], {:returned_error, :hardware_failure}} in failures
    end

    test "disarm transitions to error state when callback raises" do
      start_supervised!(RobotWithRaisingActuator)

      :ok = BB.Safety.arm(RobotWithRaisingActuator)

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithRaisingActuator)

      assert BB.Safety.state(RobotWithRaisingActuator) == :error
      assert length(failures) == 1

      assert {[:base, :joint1, :failing], {:exception, "Hardware communication failed"}} in failures
    end

    test "disarm transitions to error state when callback throws" do
      start_supervised!(RobotWithThrowingActuator)

      :ok = BB.Safety.arm(RobotWithThrowingActuator)

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithThrowingActuator)

      assert BB.Safety.state(RobotWithThrowingActuator) == :error
      assert length(failures) == 1
      assert {[:base, :joint1, :failing], {:throw, :hardware_timeout}} in failures
    end
  end

  describe "cannot arm in error state" do
    test "arm returns error when robot is in error state" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert {:error, :in_error} = BB.Safety.arm(RobotWithFailingActuator)
    end
  end

  describe "force_disarm/1" do
    test "resets error state to disarmed" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      assert BB.Safety.state(RobotWithFailingActuator) == :error

      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert BB.Safety.state(RobotWithFailingActuator) == :disarmed
      assert BB.Safety.in_error?(RobotWithFailingActuator) == false
    end

    test "allows arming after force_disarm" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert BB.Safety.armed?(RobotWithFailingActuator) == true
    end

    test "returns error when not in error state" do
      start_supervised!(RobotWithFailingActuator)

      assert {:error, :not_in_error} = BB.Safety.force_disarm(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert {:error, :not_in_error} = BB.Safety.force_disarm(RobotWithFailingActuator)
    end
  end

  describe "pubsub transitions for error state" do
    test "publishes error transition when disarm fails" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)

      BB.PubSub.subscribe(RobotWithFailingActuator, [:state_machine])

      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :disarming, to: :error}}}
    end

    test "publishes disarmed transition on force_disarm" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      BB.PubSub.subscribe(RobotWithFailingActuator, [:state_machine])

      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :error, to: :disarmed}}}
    end
  end

  describe "disarming state" do
    test "publishes disarming transition before callbacks run" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      BB.PubSub.subscribe(RobotWithFailingActuator, [:state_machine])

      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :armed, to: :disarming}}}
    end

    test "disarming?/1 returns true while disarming" do
      start_supervised!(RobotWithFailingActuator)

      assert BB.Safety.disarming?(RobotWithFailingActuator) == false

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert BB.Safety.disarming?(RobotWithFailingActuator) == false

      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      assert BB.Safety.disarming?(RobotWithFailingActuator) == false
    end

    test "double disarm returns error" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert {:error, :already_in_error} = BB.Safety.disarm(RobotWithFailingActuator)
    end
  end

  describe "timeout handling" do
    @tag timeout: 10_000
    test "slow callback times out and transitions to error state" do
      start_supervised!(RobotWithSlowActuator)

      :ok = BB.Safety.arm(RobotWithSlowActuator)

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithSlowActuator)

      assert BB.Safety.state(RobotWithSlowActuator) == :error
      assert length(failures) == 1
      assert {:unknown, {:timeout, 5000}} in failures
    end
  end
end
