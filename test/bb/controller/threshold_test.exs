# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.ThresholdTest do
  use ExUnit.Case, async: true

  alias BB.Controller.Action
  alias BB.Controller.Threshold

  defmodule TestRobot do
    @moduledoc false

    def robot, do: %BB.Robot{}

    def disarm(_goal) do
      send(self(), :disarm_called)
      {:ok, :disarmed}
    end
  end

  describe "init/1 validation" do
    test "raises when neither min nor max is provided" do
      opts = [
        bb: %{robot: TestRobot, path: [:controller, :test]},
        topic: [:sensor, :test],
        field: :value,
        action: Action.command(:disarm),
        cooldown_ms: 1000
      ]

      assert_raise ArgumentError,
                   "BB.Controller.Threshold requires at least one of :min or :max",
                   fn ->
                     Threshold.init(opts)
                   end
    end
  end

  describe "threshold checking via handle_info delegation" do
    defp build_state(field, min, max) do
      match_fn = fn msg ->
        value = get_field(msg.payload, field)
        threshold_exceeded?(value, min, max)
      end

      %{
        opts: [
          bb: %{robot: TestRobot, path: [:controller, :test]},
          topic: [:sensor, :test],
          field: field,
          min: min,
          max: max,
          match: match_fn,
          action: Action.command(:disarm),
          cooldown_ms: 0
        ],
        last_triggered: :never
      }
    end

    defp get_field(payload, field) when is_atom(field), do: Map.get(payload, field)
    defp get_field(payload, path) when is_list(path), do: get_in(payload, path)

    defp threshold_exceeded?(nil, _min, _max), do: false

    defp threshold_exceeded?(value, min, max) do
      below_min? = if min, do: value < min, else: false
      above_max? = if max, do: value > max, else: false
      below_min? or above_max?
    end

    defp build_message(payload) do
      %BB.Message{
        timestamp: System.monotonic_time(:nanosecond),
        frame_id: :sensor,
        payload: payload
      }
    end

    test "triggers when value exceeds max" do
      state = build_state(:current, nil, 1.0)
      msg = build_message(%{current: 1.5})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_received :disarm_called
    end

    test "does not trigger when value is within max" do
      state = build_state(:current, nil, 1.0)
      msg = build_message(%{current: 0.5})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      refute_received :disarm_called
    end

    test "triggers when value falls below min" do
      state = build_state(:temperature, 10.0, nil)
      msg = build_message(%{temperature: 5.0})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_received :disarm_called
    end

    test "does not trigger when value is within min" do
      state = build_state(:temperature, 10.0, nil)
      msg = build_message(%{temperature: 15.0})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      refute_received :disarm_called
    end

    test "triggers when value is outside both bounds (max)" do
      state = build_state(:value, 10.0, 90.0)
      msg = build_message(%{value: 95.0})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_received :disarm_called
    end

    test "triggers when value is outside both bounds (min)" do
      state = build_state(:value, 10.0, 90.0)
      msg = build_message(%{value: 5.0})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_received :disarm_called
    end

    test "does not trigger when value is within both bounds" do
      state = build_state(:value, 10.0, 90.0)
      msg = build_message(%{value: 50.0})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      refute_received :disarm_called
    end

    test "supports nested field paths" do
      state = build_state([:sensor, :current], nil, 1.0)
      msg = build_message(%{sensor: %{current: 1.5}})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      assert_received :disarm_called
    end

    test "does not trigger when field is nil" do
      state = build_state(:missing_field, nil, 1.0)
      msg = build_message(%{other_field: 1.5})

      {:noreply, _new_state} = Threshold.handle_info({:bb, [:sensor, :test], msg}, state)

      refute_received :disarm_called
    end
  end
end
