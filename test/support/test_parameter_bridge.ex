# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.ParameterBridge do
  @moduledoc """
  Reference implementation of `BB.Parameter.Protocol` for testing.

  Records all calls for test assertions and provides controllable responses.

  ## Usage

      # Start robot with bridge
      start_supervised!(RobotWithBridge)
      bridge_pid = BB.Process.whereis(RobotWithBridge, :test_bridge)

      # Register to receive change notifications
      BB.Test.ParameterBridge.register_test_process(bridge_pid, self())

      # Simulate inbound requests from remote
      {:ok, params} = BB.Test.ParameterBridge.list_params(bridge_pid)
      {:ok, value} = BB.Test.ParameterBridge.get_param(bridge_pid, [:speed])
      :ok = BB.Test.ParameterBridge.set_param(bridge_pid, [:speed], 2.0)

      # Receive outbound change notifications
      assert_receive {:bridge_change, %BB.Parameter.Changed{}}
  """

  use GenServer
  @behaviour BB.Parameter.Protocol

  defmodule RemoteParamValue do
    @moduledoc false
    defstruct [:value]
  end

  defstruct [:robot, :test_pid, :calls, :remote_params, :subscriptions]

  @type t :: %__MODULE__{
          robot: module(),
          test_pid: pid() | nil,
          calls: [{atom(), list()}],
          remote_params: %{BB.Parameter.Protocol.param_id() => term()},
          subscriptions: MapSet.t(BB.Parameter.Protocol.param_id())
        }

  # Client API

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  @doc "List all parameters (simulates inbound request from remote)."
  def list_params(pid, prefix \\ nil) do
    GenServer.call(pid, {:list_params, prefix})
  end

  @doc "Get a parameter value (simulates inbound request from remote)."
  def get_param(pid, path) do
    GenServer.call(pid, {:get_param, path})
  end

  @doc "Set a parameter value (simulates inbound request from remote)."
  def set_param(pid, path, value) do
    GenServer.call(pid, {:set_param, path, value})
  end

  @doc "Get all recorded calls."
  def get_calls(pid) do
    GenServer.call(pid, :get_calls)
  end

  @doc "Clear recorded calls."
  def clear_calls(pid) do
    GenServer.call(pid, :clear_calls)
  end

  @doc "Register a test process to receive change notifications."
  def register_test_process(pid, test_pid) do
    GenServer.call(pid, {:register_test_process, test_pid})
  end

  @doc "Set up fake remote parameters for testing inbound remote access."
  def set_remote_params(pid, params) when is_map(params) do
    GenServer.call(pid, {:set_remote_params, params})
  end

  @doc "Simulate a remote parameter change (for testing subscriptions)."
  def simulate_remote_change(pid, param_id, value) do
    GenServer.call(pid, {:simulate_remote_change, param_id, value})
  end

  # BB.Parameter.Protocol callbacks

  @impl BB.Parameter.Protocol
  def handle_change(_robot, changed, state) do
    state = record_call(state, :handle_change, [changed])

    if state.test_pid do
      send(state.test_pid, {:bridge_change, changed})
    end

    {:ok, state}
  end

  @impl BB.Parameter.Protocol
  def list_remote(state) do
    params =
      Enum.map(state.remote_params, fn {id, value} ->
        param_atom = if is_atom(id), do: id, else: String.to_atom(id)
        %{id: id, value: value, type: nil, doc: nil, path: [:test_bridge, param_atom]}
      end)

    {:ok, params, state}
  end

  @impl BB.Parameter.Protocol
  def get_remote(param_id, state) do
    case Map.fetch(state.remote_params, param_id) do
      {:ok, value} -> {:ok, value, state}
      :error -> {:error, :not_found, state}
    end
  end

  @impl BB.Parameter.Protocol
  def set_remote(param_id, value, state) do
    state = %{state | remote_params: Map.put(state.remote_params, param_id, value)}
    {:ok, state}
  end

  @impl BB.Parameter.Protocol
  def subscribe_remote(param_id, state) do
    state = %{state | subscriptions: MapSet.put(state.subscriptions, param_id)}
    {:ok, state}
  end

  # GenServer callbacks

  @impl GenServer
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    user_opts = Keyword.delete(opts, :bb)

    BB.PubSub.subscribe(robot, [:param])

    state = %__MODULE__{
      robot: robot,
      test_pid: Keyword.get(user_opts, :test_pid),
      calls: [],
      remote_params: Keyword.get(user_opts, :remote_params, %{}),
      subscriptions: MapSet.new()
    }

    {:ok, state}
  end

  @impl GenServer
  def handle_call(:get_calls, _from, state) do
    {:reply, Enum.reverse(state.calls), state}
  end

  def handle_call(:clear_calls, _from, state) do
    {:reply, :ok, %{state | calls: []}}
  end

  def handle_call({:register_test_process, test_pid}, _from, state) do
    {:reply, :ok, %{state | test_pid: test_pid}}
  end

  def handle_call({:list_params, prefix}, _from, state) do
    state = record_call(state, :list_params, [prefix])
    list_prefix = prefix || []
    result = BB.Parameter.list(state.robot, prefix: list_prefix)
    {:reply, {:ok, result}, state}
  end

  def handle_call({:get_param, path}, _from, state) do
    state = record_call(state, :get_param, [path])
    result = BB.Parameter.get(state.robot, path)
    {:reply, result, state}
  end

  def handle_call({:set_param, path, value}, _from, state) do
    state = record_call(state, :set_param, [path, value])
    result = BB.Parameter.set(state.robot, path, value)
    {:reply, result, state}
  end

  # Client API handlers for test setup
  def handle_call({:set_remote_params, params}, _from, state) do
    {:reply, :ok, %{state | remote_params: params}}
  end

  def handle_call({:simulate_remote_change, param_id, value}, _from, state) do
    state = %{state | remote_params: Map.put(state.remote_params, param_id, value)}

    if MapSet.member?(state.subscriptions, param_id) do
      # Convert param_id to atom for PubSub path compatibility
      param_atom = if is_atom(param_id), do: param_id, else: String.to_atom(param_id)

      message = %BB.Message{
        timestamp: System.monotonic_time(:nanosecond),
        frame_id: :remote,
        payload: %RemoteParamValue{value: value}
      }

      BB.PubSub.publish(state.robot, [:test_bridge, param_atom], message)
    end

    {:reply, :ok, state}
  end

  # Remote parameter access handlers (called by BB.Parameter.list_remote etc)
  def handle_call(:list_remote, _from, state) do
    state = record_call(state, :list_remote, [])
    {:ok, params, state} = list_remote(state)
    {:reply, {:ok, params}, state}
  end

  def handle_call({:get_remote, param_id}, _from, state) do
    state = record_call(state, :get_remote, [param_id])

    case get_remote(param_id, state) do
      {:ok, value, state} -> {:reply, {:ok, value}, state}
      {:error, reason, state} -> {:reply, {:error, reason}, state}
    end
  end

  def handle_call({:set_remote, param_id, value}, _from, state) do
    state = record_call(state, :set_remote, [param_id, value])
    {:ok, state} = set_remote(param_id, value, state)
    {:reply, :ok, state}
  end

  def handle_call({:subscribe_remote, param_id}, _from, state) do
    state = record_call(state, :subscribe_remote, [param_id])
    {:ok, state} = subscribe_remote(param_id, state)
    {:reply, :ok, state}
  end

  @impl GenServer
  def handle_info({:bb, [:param | _path], message}, state) do
    {:ok, new_state} = handle_change(state.robot, message.payload, state)
    {:noreply, new_state}
  end

  def handle_info(_msg, state) do
    {:noreply, state}
  end

  # Private

  defp record_call(state, function, args) do
    %{state | calls: [{function, args} | state.calls]}
  end
end
