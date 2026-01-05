# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.ActionTest do
  use ExUnit.Case, async: true

  alias BB.Controller.Action
  alias BB.Controller.Action.{Callback, Command, Context}

  describe "command/1" do
    test "creates Command struct with command name" do
      action = Action.command(:disarm)

      assert %Command{command: :disarm, args: []} = action
    end
  end

  describe "command/2" do
    test "creates Command struct with command name and args" do
      action = Action.command(:move_to, target: :home, speed: 0.5)

      assert %Command{command: :move_to, args: [target: :home, speed: 0.5]} = action
    end
  end

  describe "handle_event/1" do
    test "creates Callback struct with handler function" do
      handler = fn _msg, _ctx -> :ok end
      action = Action.handle_event(handler)

      assert %Callback{handler: ^handler} = action
    end

    test "rejects functions with wrong arity" do
      assert_raise FunctionClauseError, fn ->
        Action.handle_event(fn _msg -> :ok end)
      end
    end
  end

  describe "execute/3 with Command" do
    defmodule TestRobot do
      def robot, do: %BB.Robot{}

      def disarm(_goal) do
        send(self(), {:command_called, :disarm})
        {:ok, :disarmed}
      end

      def move_to(goal) do
        send(self(), {:command_called, :move_to, goal})
        {:ok, :moved}
      end
    end

    test "invokes command on robot module" do
      action = %Command{command: :disarm, args: []}
      context = %Context{robot_module: TestRobot}
      message = %BB.Message{timestamp: 0, frame_id: :test, payload: %{}}

      result = Action.execute(action, message, context)

      assert result == {:ok, :disarmed}
      assert_received {:command_called, :disarm}
    end

    test "passes args as map to command" do
      action = %Command{command: :move_to, args: [target: :home]}
      context = %Context{robot_module: TestRobot}
      message = %BB.Message{timestamp: 0, frame_id: :test, payload: %{}}

      result = Action.execute(action, message, context)

      assert result == {:ok, :moved}
      assert_received {:command_called, :move_to, %{target: :home}}
    end
  end

  describe "execute/3 with Callback" do
    test "calls handler with message and context" do
      test_pid = self()

      handler = fn msg, ctx ->
        send(test_pid, {:handler_called, msg, ctx})
        :callback_result
      end

      action = %Callback{handler: handler}
      context = %Context{robot_module: SomeRobot, controller_name: :test_controller}
      message = %BB.Message{timestamp: 123, frame_id: :test, payload: %{value: 42}}

      result = Action.execute(action, message, context)

      assert result == :callback_result
      assert_received {:handler_called, ^message, ^context}
    end
  end
end
