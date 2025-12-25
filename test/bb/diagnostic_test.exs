# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.DiagnosticTest do
  use ExUnit.Case, async: true

  alias BB.Diagnostic

  describe "new/1" do
    test "creates diagnostic with required fields" do
      diagnostic =
        Diagnostic.new(
          component: [:robot, :arm],
          level: :ok,
          message: "Arm ready"
        )

      assert diagnostic.component == [:robot, :arm]
      assert diagnostic.level == :ok
      assert diagnostic.message == "Arm ready"
      assert diagnostic.values == %{}
      assert %DateTime{} = diagnostic.timestamp
    end

    test "creates diagnostic with optional values" do
      diagnostic =
        Diagnostic.new(
          component: [:robot, :motor],
          level: :warn,
          message: "Temperature high",
          values: %{temperature: 65.0, threshold: 70.0}
        )

      assert diagnostic.values == %{temperature: 65.0, threshold: 70.0}
    end

    test "creates diagnostic with custom timestamp" do
      timestamp = ~U[2025-01-15 10:30:00Z]

      diagnostic =
        Diagnostic.new(
          component: [:robot],
          level: :ok,
          message: "OK",
          timestamp: timestamp
        )

      assert diagnostic.timestamp == timestamp
    end

    test "raises on missing required fields" do
      assert_raise KeyError, fn ->
        Diagnostic.new(level: :ok, message: "Missing component")
      end

      assert_raise KeyError, fn ->
        Diagnostic.new(component: [:robot], message: "Missing level")
      end

      assert_raise KeyError, fn ->
        Diagnostic.new(component: [:robot], level: :ok)
      end
    end
  end

  describe "publish/1" do
    setup do
      test_pid = self()

      :telemetry.attach(
        "test-diagnostic-handler-#{inspect(test_pid)}",
        [:bb, :diagnostic],
        fn event, measurements, metadata, _config ->
          send(test_pid, {:telemetry_event, event, measurements, metadata})
        end,
        nil
      )

      on_exit(fn ->
        :telemetry.detach("test-diagnostic-handler-#{inspect(test_pid)}")
      end)

      :ok
    end

    test "publishes diagnostic struct via telemetry" do
      diagnostic =
        Diagnostic.new(
          component: [:robot, :gripper],
          level: :warn,
          message: "Grip force low",
          values: %{force: 5.0}
        )

      :ok = Diagnostic.publish(diagnostic)

      assert_receive {:telemetry_event, [:bb, :diagnostic], %{}, received_diagnostic}
      assert received_diagnostic.component == [:robot, :gripper]
      assert received_diagnostic.level == :warn
      assert received_diagnostic.message == "Grip force low"
      assert received_diagnostic.values == %{force: 5.0}
    end

    test "publishes from keyword options" do
      :ok =
        Diagnostic.publish(
          component: [:robot],
          level: :ok,
          message: "All systems nominal"
        )

      assert_receive {:telemetry_event, [:bb, :diagnostic], %{}, diagnostic}
      assert diagnostic.level == :ok
    end
  end

  describe "convenience functions" do
    setup do
      test_pid = self()

      :telemetry.attach(
        "test-convenience-handler-#{inspect(test_pid)}",
        [:bb, :diagnostic],
        fn _event, _measurements, metadata, _config ->
          send(test_pid, {:diagnostic, metadata})
        end,
        nil
      )

      on_exit(fn ->
        :telemetry.detach("test-convenience-handler-#{inspect(test_pid)}")
      end)

      :ok
    end

    test "ok/3 publishes :ok diagnostic" do
      :ok = Diagnostic.ok([:robot], "Ready")

      assert_receive {:diagnostic, diagnostic}
      assert diagnostic.level == :ok
      assert diagnostic.component == [:robot]
      assert diagnostic.message == "Ready"
    end

    test "warn/3 publishes :warn diagnostic" do
      :ok = Diagnostic.warn([:robot, :motor], "Hot", values: %{temp: 60})

      assert_receive {:diagnostic, diagnostic}
      assert diagnostic.level == :warn
      assert diagnostic.values == %{temp: 60}
    end

    test "error/3 publishes :error diagnostic" do
      :ok = Diagnostic.error([:robot, :sensor], "Disconnected")

      assert_receive {:diagnostic, diagnostic}
      assert diagnostic.level == :error
    end

    test "stale/3 publishes :stale diagnostic" do
      :ok = Diagnostic.stale([:robot, :camera], "No frames")

      assert_receive {:diagnostic, diagnostic}
      assert diagnostic.level == :stale
    end
  end
end
