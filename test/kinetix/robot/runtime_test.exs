# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.RuntimeTest do
  use ExUnit.Case, async: true
  alias Kinetix.Robot.Runtime

  defmodule TestRobot do
    @moduledoc false
    use Kinetix

    robot do
      link :base_link do
      end
    end
  end

  defmodule RobotWithCommands do
    @moduledoc false
    use Kinetix

    robot do
      commands do
        command :immediate do
          handler Kinetix.Test.ImmediateSuccessCommand
          allowed_states [:idle]
        end

        command :async_cmd do
          handler Kinetix.Test.AsyncCommand
          allowed_states [:idle]
        end

        command :rejecting do
          handler Kinetix.Test.RejectingCommand
          allowed_states [:idle]
        end

        command :preemptable do
          handler Kinetix.Test.AsyncCommand
          allowed_states [:idle, :executing]
        end
      end

      link :base_link do
      end
    end
  end

  describe "state machine lifecycle" do
    test "robot starts in disarmed state by default" do
      start_supervised!(TestRobot)

      assert Runtime.state(TestRobot) == :disarmed
    end

    test "can transition to idle state" do
      start_supervised!(TestRobot)

      assert {:ok, :idle} = Runtime.transition(TestRobot, :idle)
      assert Runtime.state(TestRobot) == :idle
    end

    test "can transition to executing state" do
      start_supervised!(TestRobot)

      {:ok, :idle} = Runtime.transition(TestRobot, :idle)
      {:ok, :executing} = Runtime.transition(TestRobot, :executing)
      assert Runtime.state(TestRobot) == :executing
    end

    test "can transition back to disarmed" do
      start_supervised!(TestRobot)

      {:ok, :idle} = Runtime.transition(TestRobot, :idle)
      {:ok, :disarmed} = Runtime.transition(TestRobot, :disarmed)
      assert Runtime.state(TestRobot) == :disarmed
    end
  end

  describe "check_allowed/2" do
    test "returns :ok when current state is in allowed list" do
      start_supervised!(TestRobot)

      assert :ok = Runtime.check_allowed(TestRobot, [:disarmed, :idle])
    end

    test "returns error when current state is not in allowed list" do
      start_supervised!(TestRobot)

      assert {:error, %Runtime.StateError{}} =
               Runtime.check_allowed(TestRobot, [:idle, :executing])
    end

    test "error contains current state and allowed states" do
      start_supervised!(TestRobot)

      {:error, error} = Runtime.check_allowed(TestRobot, [:idle, :executing])

      assert error.current_state == :disarmed
      assert error.allowed_states == [:idle, :executing]
    end
  end

  describe "state transition with idle state" do
    test "check_allowed works after transition" do
      start_supervised!(TestRobot)

      {:ok, :idle} = Runtime.transition(TestRobot, :idle)

      assert :ok = Runtime.check_allowed(TestRobot, [:idle])
      assert {:error, _} = Runtime.check_allowed(TestRobot, [:disarmed])
    end
  end

  describe "pubsub integration" do
    alias Kinetix.StateMachine.Transition

    test "publishes state transitions to pubsub" do
      start_supervised!(TestRobot)

      Kinetix.PubSub.subscribe(TestRobot, [:state_machine])

      {:ok, :idle} = Runtime.transition(TestRobot, :idle)

      assert_receive {:kinetix, [:state_machine],
                      %Kinetix.Message{payload: %Transition{from: :disarmed, to: :idle}}}
    end

    test "does not publish when state doesn't change" do
      start_supervised!(TestRobot)

      Kinetix.PubSub.subscribe(TestRobot, [:state_machine])

      {:ok, :disarmed} = Runtime.transition(TestRobot, :disarmed)

      refute_receive {:kinetix, [:state_machine], _}
    end
  end

  describe "robot state access" do
    test "can get the robot struct" do
      start_supervised!(TestRobot)

      robot = Runtime.get_robot(TestRobot)
      assert robot.name == TestRobot
    end

    test "can get the robot state (ETS)" do
      start_supervised!(TestRobot)

      robot_state = Runtime.get_robot_state(TestRobot)
      assert %Kinetix.Robot.State{} = robot_state
    end
  end

  describe "command execution" do
    test "rejects command when not in allowed state" do
      start_supervised!(RobotWithCommands)

      assert {:error, %Runtime.StateError{current_state: :disarmed}} =
               Runtime.execute(RobotWithCommands, :immediate, %{})
    end

    test "executes command that succeeds immediately" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      assert {:ok, :done} = Runtime.execute(RobotWithCommands, :immediate, %{})
    end

    test "rejects unknown commands" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      assert {:error, {:unknown_command, :nonexistent}} =
               Runtime.execute(RobotWithCommands, :nonexistent, %{})
    end

    test "handler can reject goal" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      assert {:error, {:rejected, :not_allowed}} =
               Runtime.execute(RobotWithCommands, :rejecting, %{})
    end

    test "async command transitions state to executing" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _ref} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})

      assert_receive :executing
      assert Runtime.state(RobotWithCommands) == :executing
    end

    test "async command completes via handle_info" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _ref} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing

      runtime_pid = GenServer.whereis(Runtime.via(RobotWithCommands))
      send(runtime_pid, :complete)

      Process.sleep(10)
      assert Runtime.state(RobotWithCommands) == :idle
    end

    test "command can preempt executing command" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _ref} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing
      assert Runtime.state(RobotWithCommands) == :executing

      {:ok, _ref2} = Runtime.execute(RobotWithCommands, :preemptable, %{notify: self()})
      assert_receive :executing
    end

    test "non-preemptable command rejected when executing" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _ref} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing

      assert {:error, %Runtime.StateError{current_state: :executing}} =
               Runtime.execute(RobotWithCommands, :immediate, %{})
    end

    test "cancel returns error when nothing executing" do
      start_supervised!(RobotWithCommands)

      assert {:error, :no_execution} = Runtime.cancel(RobotWithCommands)
    end
  end

  describe "generated command functions" do
    test "robot module has generated command functions" do
      assert function_exported?(RobotWithCommands, :immediate, 0)
      assert function_exported?(RobotWithCommands, :immediate, 1)
      assert function_exported?(RobotWithCommands, :async_cmd, 0)
      assert function_exported?(RobotWithCommands, :async_cmd, 1)
    end

    test "generated functions execute commands" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      assert {:ok, :done} = RobotWithCommands.immediate()
    end

    test "generated functions pass goals to handler" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _ref} = RobotWithCommands.async_cmd(notify: self())
      assert_receive :executing
    end
  end
end
