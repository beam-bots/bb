# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.StateMachineTest do
  use ExUnit.Case, async: true
  alias Kinetix.StateMachine

  defmodule TestRobot do
    @moduledoc false
    use Kinetix

    robot do
      link :base_link do
      end
    end
  end

  describe "state machine lifecycle" do
    test "robot starts in disarmed state by default" do
      start_supervised!(TestRobot)

      assert StateMachine.state(TestRobot) == :disarmed
    end

    test "can transition to idle state" do
      start_supervised!(TestRobot)

      assert {:ok, :idle} = StateMachine.transition(TestRobot, :idle)
      assert StateMachine.state(TestRobot) == :idle
    end

    test "can transition to executing state" do
      start_supervised!(TestRobot)

      {:ok, :idle} = StateMachine.transition(TestRobot, :idle)
      {:ok, :executing} = StateMachine.transition(TestRobot, :executing)
      assert StateMachine.state(TestRobot) == :executing
    end

    test "can transition back to disarmed" do
      start_supervised!(TestRobot)

      {:ok, :idle} = StateMachine.transition(TestRobot, :idle)
      {:ok, :disarmed} = StateMachine.transition(TestRobot, :disarmed)
      assert StateMachine.state(TestRobot) == :disarmed
    end
  end

  describe "check_allowed/2" do
    test "returns :ok when current state is in allowed list" do
      start_supervised!(TestRobot)

      # Default state is :disarmed
      assert :ok = StateMachine.check_allowed(TestRobot, [:disarmed, :idle])
    end

    test "returns error when current state is not in allowed list" do
      start_supervised!(TestRobot)

      # Default state is :disarmed
      assert {:error, %StateMachine.StateError{}} =
               StateMachine.check_allowed(TestRobot, [:idle, :executing])
    end

    test "error contains current state and allowed states" do
      start_supervised!(TestRobot)

      {:error, error} = StateMachine.check_allowed(TestRobot, [:idle, :executing])

      assert error.current_state == :disarmed
      assert error.allowed_states == [:idle, :executing]
    end
  end

  describe "state transition with idle state" do
    test "check_allowed works after transition" do
      start_supervised!(TestRobot)

      {:ok, :idle} = StateMachine.transition(TestRobot, :idle)

      assert :ok = StateMachine.check_allowed(TestRobot, [:idle])
      assert {:error, _} = StateMachine.check_allowed(TestRobot, [:disarmed])
    end
  end

  describe "pubsub integration" do
    alias Kinetix.StateMachine.Transition

    test "publishes state transitions to pubsub" do
      start_supervised!(TestRobot)

      # Subscribe to state machine events
      Kinetix.PubSub.subscribe(TestRobot, [:state_machine])

      {:ok, :idle} = StateMachine.transition(TestRobot, :idle)

      assert_receive {:kinetix, [:state_machine],
                      %Kinetix.Message{payload: %Transition{from: :disarmed, to: :idle}}}
    end

    test "does not publish when state doesn't change" do
      start_supervised!(TestRobot)

      Kinetix.PubSub.subscribe(TestRobot, [:state_machine])

      # Transition to same state
      {:ok, :disarmed} = StateMachine.transition(TestRobot, :disarmed)

      refute_receive {:kinetix, [:state_machine], _}
    end
  end
end
