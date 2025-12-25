# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.TelemetryTest do
  use ExUnit.Case, async: true

  alias BB.Telemetry

  describe "span/3" do
    test "emits start and stop events" do
      test_pid = self()

      handler = fn event, measurements, metadata, _config ->
        send(test_pid, {:telemetry, event, measurements, metadata})
      end

      :telemetry.attach_many(
        "test-span-#{inspect(test_pid)}",
        [
          [:test, :span, :start],
          [:test, :span, :stop]
        ],
        handler,
        nil
      )

      on_exit(fn ->
        :telemetry.detach("test-span-#{inspect(test_pid)}")
      end)

      result = Telemetry.span([:test, :span], %{key: :value}, fn -> {:result, %{extra: 1}} end)

      assert result == :result

      assert_receive {:telemetry, [:test, :span, :start], start_measurements, _start_metadata}
      assert Map.has_key?(start_measurements, :system_time)

      assert_receive {:telemetry, [:test, :span, :stop], stop_measurements, _stop_metadata}
      assert Map.has_key?(stop_measurements, :duration)
    end

    test "returns the result from the span function" do
      result = Telemetry.span([:test, :return], %{}, fn -> {{:ok, 42}, %{}} end)
      assert result == {:ok, 42}
    end
  end

  describe "emit/3" do
    test "emits telemetry event" do
      test_pid = self()

      :telemetry.attach(
        "test-emit-#{inspect(test_pid)}",
        [:test, :emit],
        fn event, measurements, metadata, _config ->
          send(test_pid, {:telemetry, event, measurements, metadata})
        end,
        nil
      )

      on_exit(fn ->
        :telemetry.detach("test-emit-#{inspect(test_pid)}")
      end)

      :ok = Telemetry.emit([:test, :emit], %{count: 42}, %{source: :test})

      assert_receive {:telemetry, [:test, :emit], %{count: 42}, %{source: :test}}
    end
  end
end
