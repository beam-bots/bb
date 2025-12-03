# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.RuntimeTest do
  use ExUnit.Case, async: true
  alias Kinetix.Robot.Runtime

  defmodule TestRobot do
    @moduledoc false
    use Kinetix

    topology do
      link :base_link do
      end
    end
  end

  defmodule RobotWithCommands do
    @moduledoc false
    use Kinetix

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

    topology do
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

      # Errors now come through the awaited task
      {:ok, task} = Runtime.execute(RobotWithCommands, :immediate, %{})

      assert {:error, %Runtime.StateError{current_state: :disarmed}} = Task.await(task)
    end

    test "executes command that succeeds immediately" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task} = Runtime.execute(RobotWithCommands, :immediate, %{})
      assert {:ok, :done} = Task.await(task)
    end

    test "rejects unknown commands" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      # Errors now come through the awaited task
      {:ok, task} = Runtime.execute(RobotWithCommands, :nonexistent, %{})
      assert {:error, {:unknown_command, :nonexistent}} = Task.await(task)
    end

    test "handler can return error" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task} = Runtime.execute(RobotWithCommands, :rejecting, %{})
      assert {:error, :not_allowed} = Task.await(task)
    end

    test "async command transitions state to executing" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _task} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})

      assert_receive :executing
      assert Runtime.state(RobotWithCommands) == :executing
    end

    test "async command transitions back to idle on completion" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing

      assert {:ok, :completed} = Task.await(task)

      # Give the runtime a moment to process the :DOWN message
      Process.sleep(10)
      assert Runtime.state(RobotWithCommands) == :idle
    end

    test "command can preempt executing command" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task1} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing
      assert Runtime.state(RobotWithCommands) == :executing

      # Start a preemptable command - it should cancel the first task
      {:ok, task2} = Runtime.execute(RobotWithCommands, :preemptable, %{notify: self()})
      assert_receive :executing

      # First task returns cancelled error
      assert {:error, :cancelled} = Task.await(task1)

      # Second task should complete
      assert {:ok, :completed} = Task.await(task2)
    end

    test "non-preemptable command rejected when executing" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, _task} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing

      # Errors come through the task now
      {:ok, task} = Runtime.execute(RobotWithCommands, :immediate, %{})
      assert {:error, %Runtime.StateError{current_state: :executing}} = Task.await(task)
    end

    test "cancel returns error when nothing executing" do
      start_supervised!(RobotWithCommands)

      assert {:error, :no_execution} = Runtime.cancel(RobotWithCommands)
    end

    test "cancel terminates running command" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task} = Runtime.execute(RobotWithCommands, :async_cmd, %{notify: self()})
      assert_receive :executing

      assert :ok = Runtime.cancel(RobotWithCommands)

      # Task returns cancelled error
      assert {:error, :cancelled} = Task.await(task)

      # State transitions to idle synchronously on cancel
      assert Runtime.state(RobotWithCommands) == :idle
    end
  end

  describe "generated command functions" do
    test "robot module has generated command functions" do
      assert function_exported?(RobotWithCommands, :immediate, 0)
      assert function_exported?(RobotWithCommands, :immediate, 1)
      assert function_exported?(RobotWithCommands, :async_cmd, 0)
      assert function_exported?(RobotWithCommands, :async_cmd, 1)
    end

    test "generated functions execute commands and return tasks" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task} = RobotWithCommands.immediate()
      assert {:ok, :done} = Task.await(task)
    end

    test "generated functions pass goals to handler" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      {:ok, task} = RobotWithCommands.async_cmd(notify: self())
      assert_receive :executing
      assert {:ok, :completed} = Task.await(task)
    end
  end

  describe "command events" do
    alias Kinetix.Command.Event

    test "broadcasts command started event" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      Kinetix.PubSub.subscribe(RobotWithCommands, [:command, :immediate])
      {:ok, task} = Runtime.execute(RobotWithCommands, :immediate, %{})

      assert_receive {:kinetix, [:command, :immediate, _ref],
                      %Kinetix.Message{payload: %Event{status: :started}}}

      Task.await(task)
    end

    test "broadcasts command succeeded event" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      Kinetix.PubSub.subscribe(RobotWithCommands, [:command, :immediate])
      {:ok, task} = Runtime.execute(RobotWithCommands, :immediate, %{})
      Task.await(task)

      assert_receive {:kinetix, [:command, :immediate, _ref],
                      %Kinetix.Message{
                        payload: %Event{status: :succeeded, data: %{result: :done}}
                      }}
    end

    test "broadcasts command failed event" do
      start_supervised!(RobotWithCommands)

      {:ok, :idle} = Runtime.transition(RobotWithCommands, :idle)

      Kinetix.PubSub.subscribe(RobotWithCommands, [:command, :rejecting])
      {:ok, task} = Runtime.execute(RobotWithCommands, :rejecting, %{})
      Task.await(task)

      assert_receive {:kinetix, [:command, :rejecting, _ref],
                      %Kinetix.Message{
                        payload: %Event{status: :failed, data: %{reason: :not_allowed}}
                      }}
    end
  end
end
