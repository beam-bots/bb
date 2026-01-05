# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.PatternMatchTest do
  use ExUnit.Case, async: true

  alias BB.Controller.Action
  alias BB.Controller.PatternMatch

  defmodule TestRobot do
    @moduledoc false

    def robot, do: %BB.Robot{}

    def disarm(_goal) do
      send(self(), :disarm_called)
      {:ok, :disarmed}
    end

    def test_action(_goal) do
      send(self(), :action_triggered)
      {:ok, :done}
    end
  end

  defp build_state(opts) do
    defaults = [
      bb: %{robot: TestRobot, path: [:controller, :test]},
      topic: [:sensor, :test],
      match: fn _msg -> true end,
      action: Action.command(:disarm),
      cooldown_ms: 1000
    ]

    %{
      opts: Keyword.merge(defaults, opts),
      last_triggered: :never
    }
  end

  defp build_message(payload) do
    %BB.Message{
      timestamp: System.monotonic_time(:nanosecond),
      frame_id: :sensor,
      payload: payload
    }
  end

  describe "handle_info/2 with matching message" do
    test "triggers action when match returns true" do
      state =
        build_state(
          match: fn msg -> msg.payload.distance < 0.1 end,
          action: Action.command(:disarm),
          cooldown_ms: 0
        )

      msg = build_message(%{distance: 0.05})

      {:noreply, _new_state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_received :disarm_called
    end

    test "does not trigger action when match returns false" do
      state =
        build_state(
          match: fn msg -> msg.payload.distance < 0.1 end,
          action: Action.command(:disarm),
          cooldown_ms: 0
        )

      msg = build_message(%{distance: 0.5})

      {:noreply, _new_state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)

      refute_received :disarm_called
    end

    test "updates last_triggered timestamp after triggering" do
      state = build_state(match: fn _msg -> true end, cooldown_ms: 0)

      assert state.last_triggered == :never

      msg = build_message(%{})

      {:noreply, new_state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)

      assert is_integer(new_state.last_triggered)
    end

    test "calls callback action with message and context" do
      test_pid = self()

      callback = fn msg, ctx ->
        send(test_pid, {:callback_called, msg, ctx})
        :ok
      end

      state =
        build_state(
          match: fn _msg -> true end,
          action: Action.handle_event(callback),
          cooldown_ms: 0
        )

      msg = build_message(%{value: 42})

      {:noreply, _new_state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_receive {:callback_called, ^msg, context}
      assert context.robot_module == TestRobot
      assert context.controller_name == :test
    end
  end

  describe "cooldown behaviour" do
    test "respects cooldown between triggers" do
      state =
        build_state(
          match: fn _msg -> true end,
          action: Action.command(:test_action),
          cooldown_ms: 100
        )

      msg = build_message(%{})

      {:noreply, state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)
      assert_received :action_triggered

      {:noreply, _state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)
      refute_received :action_triggered
    end

    test "triggers again after cooldown elapsed" do
      state =
        build_state(
          match: fn _msg -> true end,
          action: Action.command(:test_action),
          cooldown_ms: 10
        )

      msg = build_message(%{})

      {:noreply, state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)
      assert_received :action_triggered

      Process.sleep(15)

      {:noreply, _state} = PatternMatch.handle_info({:bb, [:sensor, :test], msg}, state)
      assert_received :action_triggered
    end
  end

  describe "handle_info/2 with non-BB messages" do
    test "ignores non-BB messages" do
      state = build_state([])

      {:noreply, new_state} = PatternMatch.handle_info(:some_other_message, state)

      assert new_state == state
    end
  end
end
