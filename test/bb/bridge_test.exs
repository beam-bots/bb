# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.BridgeTest do
  use ExUnit.Case, async: false

  alias BB.Parameter
  alias BB.Test.ParameterBridge

  defmodule RobotWithBridge do
    @moduledoc false
    use BB

    parameters do
      param :speed, type: :float, default: 1.0
      param :enabled, type: :boolean, default: true

      group :motion do
        param :max_velocity, type: :float, default: 2.0
      end

      bridge(:test_bridge, {BB.Test.ParameterBridge, []})
    end

    topology do
      link :base
    end
  end

  defmodule RobotWithMultipleBridges do
    @moduledoc false
    use BB

    parameters do
      param :value, type: :integer, default: 42

      bridge(:bridge_a, {BB.Test.ParameterBridge, []})
      bridge(:bridge_b, {BB.Test.ParameterBridge, []})
    end

    topology do
      link :base
    end
  end

  describe "bridge supervision" do
    test "bridges are started with the robot" do
      start_supervised!(RobotWithBridge)

      bridge_pid = BB.Process.whereis(RobotWithBridge, :test_bridge)
      assert is_pid(bridge_pid)
      assert Process.alive?(bridge_pid)
    end

    test "multiple bridges can be defined" do
      start_supervised!(RobotWithMultipleBridges)

      bridge_a = BB.Process.whereis(RobotWithMultipleBridges, :bridge_a)
      bridge_b = BB.Process.whereis(RobotWithMultipleBridges, :bridge_b)

      assert is_pid(bridge_a)
      assert is_pid(bridge_b)
      assert bridge_a != bridge_b
    end

    test "bridges are supervised under BridgeSupervisor" do
      start_supervised!(RobotWithBridge)

      bridge_sup = BB.Process.whereis(RobotWithBridge, BB.BridgeSupervisor)
      assert is_pid(bridge_sup)

      children = Supervisor.which_children(bridge_sup)
      assert length(children) == 1

      [{:test_bridge, bridge_pid, :worker, _}] = children
      assert is_pid(bridge_pid)
    end
  end

  describe "inbound requests (remote → local)" do
    setup do
      start_supervised!(RobotWithBridge)
      bridge_pid = BB.Process.whereis(RobotWithBridge, :test_bridge)
      ParameterBridge.clear_calls(bridge_pid)
      {:ok, bridge: bridge_pid}
    end

    test "list_params returns all parameters", %{bridge: bridge} do
      {:ok, params} = ParameterBridge.list_params(bridge)

      paths = Enum.map(params, fn {path, _metadata} -> path end)
      assert [:speed] in paths
      assert [:enabled] in paths
      assert [:motion, :max_velocity] in paths
    end

    test "list_params with prefix filters parameters", %{bridge: bridge} do
      {:ok, params} = ParameterBridge.list_params(bridge, [:motion])

      paths = Enum.map(params, fn {path, _metadata} -> path end)
      assert [:motion, :max_velocity] in paths
      refute [:speed] in paths
    end

    test "get_param retrieves parameter value", %{bridge: bridge} do
      {:ok, value} = ParameterBridge.get_param(bridge, [:speed])
      assert value == 1.0
    end

    test "get_param returns error for unknown parameter", %{bridge: bridge} do
      {:error, :not_found} = ParameterBridge.get_param(bridge, [:unknown])
    end

    test "set_param updates parameter value", %{bridge: bridge} do
      :ok = ParameterBridge.set_param(bridge, [:speed], 5.0)

      {:ok, value} = Parameter.get(RobotWithBridge, [:speed])
      assert value == 5.0
    end

    test "set_param validates parameter type", %{bridge: bridge} do
      {:error, _reason} = ParameterBridge.set_param(bridge, [:speed], "not a float")
    end

    test "calls are recorded", %{bridge: bridge} do
      ParameterBridge.get_param(bridge, [:speed])
      ParameterBridge.set_param(bridge, [:speed], 3.0)

      calls = ParameterBridge.get_calls(bridge)

      assert {:get_param, [[:speed]]} in calls
      assert {:set_param, [[:speed], 3.0]} in calls
    end
  end

  describe "outbound notifications (local → remote)" do
    setup do
      start_supervised!(RobotWithBridge)
      bridge_pid = BB.Process.whereis(RobotWithBridge, :test_bridge)
      ParameterBridge.register_test_process(bridge_pid, self())
      ParameterBridge.clear_calls(bridge_pid)
      {:ok, bridge: bridge_pid}
    end

    test "bridge receives local parameter changes", %{bridge: _bridge} do
      :ok = Parameter.set(RobotWithBridge, [:speed], 10.0)

      assert_receive {:bridge_change, changed}, 1000
      assert changed.path == [:speed]
      assert changed.new_value == 10.0
    end

    test "bridge receives changes from other sources", %{bridge: _bridge} do
      :ok = Parameter.set(RobotWithBridge, [:enabled], false)

      assert_receive {:bridge_change, changed}, 1000
      assert changed.path == [:enabled]
      assert changed.new_value == false
    end

    test "handle_change is recorded in calls", %{bridge: bridge} do
      :ok = Parameter.set(RobotWithBridge, [:speed], 7.0)

      assert_receive {:bridge_change, _}, 1000

      calls = ParameterBridge.get_calls(bridge)
      handle_change_calls = Enum.filter(calls, fn {name, _} -> name == :handle_change end)
      assert handle_change_calls != []
    end
  end

  describe "bidirectional sync" do
    setup do
      start_supervised!(RobotWithMultipleBridges)

      bridge_a = BB.Process.whereis(RobotWithMultipleBridges, :bridge_a)
      bridge_b = BB.Process.whereis(RobotWithMultipleBridges, :bridge_b)

      ParameterBridge.register_test_process(bridge_a, self())
      ParameterBridge.clear_calls(bridge_a)
      ParameterBridge.clear_calls(bridge_b)

      {:ok, bridge_a: bridge_a, bridge_b: bridge_b}
    end

    test "change from one bridge notifies other bridges", %{
      bridge_a: bridge_a,
      bridge_b: bridge_b
    } do
      # Simulate remote client changing value via bridge_a
      :ok = ParameterBridge.set_param(bridge_a, [:value], 100)

      # bridge_a (registered as test process) should receive the change
      assert_receive {:bridge_change, changed}, 1000
      assert changed.path == [:value]
      assert changed.new_value == 100

      # bridge_b should also have recorded a handle_change call
      Process.sleep(50)
      calls_b = ParameterBridge.get_calls(bridge_b)
      handle_change_calls = Enum.filter(calls_b, fn {name, _} -> name == :handle_change end)
      assert handle_change_calls != []
    end

    test "local changes are broadcast to all bridges", %{bridge_a: bridge_a, bridge_b: bridge_b} do
      :ok = Parameter.set(RobotWithMultipleBridges, [:value], 200)

      # Wait for PubSub delivery
      Process.sleep(50)

      calls_a = ParameterBridge.get_calls(bridge_a)
      calls_b = ParameterBridge.get_calls(bridge_b)

      handle_change_a = Enum.filter(calls_a, fn {name, _} -> name == :handle_change end)
      handle_change_b = Enum.filter(calls_b, fn {name, _} -> name == :handle_change end)

      assert handle_change_a != []
      assert handle_change_b != []
    end
  end

  describe "remote parameter access (remote → local)" do
    setup do
      start_supervised!(RobotWithBridge)
      bridge_pid = BB.Process.whereis(RobotWithBridge, :test_bridge)

      # Set up fake remote parameters
      remote_params = %{
        "PITCH_RATE_P" => 0.1,
        "PITCH_RATE_I" => 0.01,
        "ROLL_RATE_P" => 0.15
      }

      ParameterBridge.set_remote_params(bridge_pid, remote_params)
      ParameterBridge.clear_calls(bridge_pid)
      {:ok, bridge: bridge_pid}
    end

    test "list_remote returns remote parameters" do
      {:ok, params} = Parameter.list_remote(RobotWithBridge, :test_bridge)

      ids = Enum.map(params, & &1.id)
      assert "PITCH_RATE_P" in ids
      assert "PITCH_RATE_I" in ids
      assert "ROLL_RATE_P" in ids
    end

    test "get_remote retrieves remote parameter value" do
      {:ok, value} = Parameter.get_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P")
      assert value == 0.1
    end

    test "get_remote returns error for unknown parameter" do
      {:error, :not_found} = Parameter.get_remote(RobotWithBridge, :test_bridge, "UNKNOWN")
    end

    test "set_remote updates remote parameter value" do
      :ok = Parameter.set_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P", 0.2)

      {:ok, value} = Parameter.get_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P")
      assert value == 0.2
    end

    test "subscribe_remote tracks subscription", %{bridge: bridge} do
      :ok = Parameter.subscribe_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P")

      calls = ParameterBridge.get_calls(bridge)
      assert {:subscribe_remote, ["PITCH_RATE_P"]} in calls
    end

    test "subscribed remote parameter changes are published", %{bridge: bridge} do
      # Subscribe to the parameter
      :ok = Parameter.subscribe_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P")

      # Subscribe to PubSub to receive the change
      BB.PubSub.subscribe(RobotWithBridge, [:test_bridge, :PITCH_RATE_P])

      # Simulate a remote change
      ParameterBridge.simulate_remote_change(bridge, "PITCH_RATE_P", 0.25)

      assert_receive {:bb, [:test_bridge, :PITCH_RATE_P], message}, 1000
      assert message.payload.value == 0.25
    end

    test "remote calls are recorded", %{bridge: bridge} do
      Parameter.list_remote(RobotWithBridge, :test_bridge)
      Parameter.get_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P")
      Parameter.set_remote(RobotWithBridge, :test_bridge, "PITCH_RATE_P", 0.3)

      calls = ParameterBridge.get_calls(bridge)

      assert {:list_remote, []} in calls
      assert {:get_remote, ["PITCH_RATE_P"]} in calls
      assert {:set_remote, ["PITCH_RATE_P", 0.3]} in calls
    end
  end
end
