<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Parameter Bridges

In this tutorial, you'll learn how to connect your robot's parameters to remote systems using parameter bridges.

## Prerequisites

Complete [Parameters](07-parameters.md). You should understand how to define and access parameters at runtime.

## What Are Parameter Bridges?

Parameter bridges provide bidirectional access between BB and remote systems:

- **Outbound (local → remote):** Expose BB's parameters to ground control stations, web UIs, or debugging tools
- **Inbound (remote → local):** Access parameters from flight controllers, external sensors, or other systems

> **For Roboticists:** Bridges work like MAVLink's parameter protocol or ROS2's parameter services. They let you enumerate, read, write, and subscribe to parameters over any transport.

> **For Elixirists:** Bridges are GenServers that implement the `BB.Parameter.Protocol` behaviour. They're supervised by the robot and integrate with PubSub for change notifications.

## Defining Bridges in the DSL

Add bridges to your `parameters` section:

```elixir
defmodule MyRobot do
  use BB

  parameters do
    param :max_speed, type: :float, default: 1.0

    bridge :debug, {MyDebugBridge, port: 4000}
  end

  topology do
    link :base
  end
end
```

Each bridge takes:
- A name (atom) - used to identify the bridge
- A child spec - module or `{module, options}` tuple

Bridges are started as part of the robot's supervision tree.

## The Protocol Behaviour

Bridges implement `BB.Parameter.Protocol`. There are two directions:

### Outbound Callback

Handle local parameter changes and notify remote clients:

```elixir
@callback handle_change(robot :: module(), changed :: BB.Parameter.Changed.t(), state) ::
  {:ok, state}
```

Bridges should also subscribe to `[:param]` via `BB.PubSub` in their GenServer `init/1`.

### Inbound Callbacks (Optional)

Access parameters on a remote system:

```elixir
@callback list_remote(state) ::
  {:ok, [remote_param()], state} | {:error, term(), state}

@callback get_remote(param_id, state) ::
  {:ok, term(), state} | {:error, term(), state}

@callback set_remote(param_id, value :: term(), state) ::
  {:ok, state} | {:error, term(), state}

@callback subscribe_remote(param_id, state) ::
  {:ok, state} | {:error, term(), state}
```

## Implementing a Simple Bridge

Here's a bridge that logs parameter changes:

```elixir
defmodule MyDebugBridge do
  use GenServer
  @behaviour BB.Parameter.Protocol

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  # GenServer init - extract robot, subscribe to param changes
  @impl GenServer
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    BB.PubSub.subscribe(robot, [:param])

    {:ok, %{
      robot: robot,
      port: Keyword.get(opts, :port, 4000)
    }}
  end

  # Handle local parameter changes
  @impl BB.Parameter.Protocol
  def handle_change(_robot, changed, state) do
    IO.puts("[DEBUG] Parameter #{inspect(changed.path)} changed:")
    IO.puts("  Old: #{inspect(changed.old_value)}")
    IO.puts("  New: #{inspect(changed.new_value)}")

    {:ok, state}
  end

  # Receive PubSub messages and dispatch to handle_change
  @impl GenServer
  def handle_info({:bb, [:param | _path], message}, state) do
    {:ok, new_state} = handle_change(state.robot, message.payload, state)
    {:noreply, new_state}
  end

  def handle_info(_msg, state) do
    {:noreply, state}
  end
end
```

Now when parameters change, you'll see debug output:

```elixir
iex> {:ok, _} = BB.Supervisor.start_link(MyRobot)
iex> BB.Parameter.set(MyRobot, [:max_speed], 2.0)
[DEBUG] Parameter [:max_speed] changed:
  Old: 1.0
  New: 2.0
:ok
```

## Accessing Remote Parameters

Bridges can also expose parameters from remote systems. This is useful when your robot communicates with a flight controller that has its own parameters.

### Implementing Inbound Access

Add the inbound callbacks to your bridge:

```elixir
defmodule MyFlightControllerBridge do
  use GenServer
  @behaviour BB.Parameter.Protocol

  # Define a message type for remote param changes
  defmodule ParamValue do
    @behaviour BB.Message
    defstruct [:value]

    @schema Spark.Options.new!(value: [type: :any, required: true])
    @impl BB.Message
    def schema, do: @schema

    defimpl BB.Message.Payload do
      def schema(_), do: @for.schema()
    end
  end

  def start_link(opts), do: GenServer.start_link(__MODULE__, opts)

  @impl GenServer
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    BB.PubSub.subscribe(robot, [:param])
    conn = connect_to_fc(opts[:device])

    {:ok, %{
      robot: robot,
      conn: conn,
      subscriptions: MapSet.new()
    }}
  end

  @impl BB.Parameter.Protocol
  def handle_change(_robot, changed, state) do
    # Optionally sync local changes to FC
    send_param_to_fc(state.conn, changed)
    {:ok, state}
  end

  # List all parameters on the flight controller
  @impl BB.Parameter.Protocol
  def list_remote(state) do
    params = fetch_all_fc_params(state.conn)
    |> Enum.map(fn {id, value} ->
      %{
        id: id,
        value: value,
        type: nil,
        doc: nil,
        path: param_id_to_path(id)
      }
    end)

    {:ok, params, state}
  end

  # Get a specific parameter from the FC
  @impl BB.Parameter.Protocol
  def get_remote(param_id, state) do
    case fetch_fc_param(state.conn, param_id) do
      {:ok, value} -> {:ok, value, state}
      :error -> {:error, :not_found, state}
    end
  end

  # Set a parameter on the FC
  @impl BB.Parameter.Protocol
  def set_remote(param_id, value, state) do
    :ok = send_fc_param_set(state.conn, param_id, value)
    {:ok, state}
  end

  # Subscribe to FC parameter changes
  @impl BB.Parameter.Protocol
  def subscribe_remote(param_id, state) do
    state = %{state | subscriptions: MapSet.put(state.subscriptions, param_id)}
    {:ok, state}
  end

  # When FC sends a param update, publish via PubSub
  @impl GenServer
  def handle_info({:fc_param_changed, param_id, value}, state) do
    if MapSet.member?(state.subscriptions, param_id) do
      path = param_id_to_path(param_id)
      message = BB.Message.new!(ParamValue, :remote, value: value)
      BB.PubSub.publish(state.robot, path, message)
    end
    {:noreply, state}
  end

  def handle_info({:bb, [:param | _], message}, state) do
    {:ok, new_state} = handle_change(state.robot, message.payload, state)
    {:noreply, new_state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # Convert "PITCH_RATE_P" to [:fc, :pitch, :rate, :p]
  defp param_id_to_path(param_id) do
    atoms = param_id
    |> String.downcase()
    |> String.split("_")
    |> Enum.map(&String.to_atom/1)

    [:fc | atoms]
  end

  # Placeholder - implement actual FC communication
  defp connect_to_fc(_device), do: :connected
  defp fetch_all_fc_params(_conn), do: [{"PITCH_RATE_P", 0.1}, {"ROLL_RATE_P", 0.15}]
  defp fetch_fc_param(_conn, _id), do: {:ok, 0.1}
  defp send_fc_param_set(_conn, _id, _value), do: :ok
  defp send_param_to_fc(_conn, _changed), do: :ok
end
```

### Using Remote Parameters from IEx

Access remote parameters through the `BB.Parameter` API:

```elixir
iex> {:ok, _} = BB.Supervisor.start_link(MyRobot)

# List parameters on the flight controller
iex> {:ok, params} = BB.Parameter.list_remote(MyRobot, :fc)
{:ok, [
  %{id: "PITCH_RATE_P", value: 0.1, path: [:fc, :pitch, :rate, :p], ...},
  %{id: "ROLL_RATE_P", value: 0.15, path: [:fc, :roll, :rate, :p], ...}
]}

# Get a specific parameter
iex> {:ok, value} = BB.Parameter.get_remote(MyRobot, :fc, "PITCH_RATE_P")
{:ok, 0.1}

# Set a parameter on the FC
iex> :ok = BB.Parameter.set_remote(MyRobot, :fc, "PITCH_RATE_P", 0.12)
:ok

# Subscribe to changes
iex> :ok = BB.Parameter.subscribe_remote(MyRobot, :fc, "PITCH_RATE_P")
:ok

# Subscribe to PubSub using the path from list_remote
iex> BB.PubSub.subscribe(MyRobot, [:fc, :pitch, :rate, :p])
{:ok, #PID<0.234.0>}
```

## Multiple Bridges

A robot can have multiple bridges for different purposes:

```elixir
parameters do
  group :motion do
    param :max_speed, type: :float, default: 1.0
  end

  # Expose params to web UI
  bridge :web, {MyPhoenixBridge, url: "ws://localhost:4000/socket"}

  # Connect to flight controller
  bridge :fc, {MyMavlinkBridge, device: "/dev/ttyACM0"}

  # Debug logging
  bridge :debug, MyDebugBridge
end
```

Each bridge operates independently:
- Changes to local params notify all bridges
- Remote params are accessed by bridge name

## Bridge Supervision

Bridges are supervised with fault isolation. If a bridge crashes:
- Other bridges continue operating
- The crashed bridge is restarted
- Local parameters remain accessible

This is handled by `BB.BridgeSupervisor`, which is separate from sensor and controller supervisors.

## Complete Example: Mock Flight Controller

Here's a complete example with a simulated flight controller:

```elixir
defmodule MockFCBridge do
  @moduledoc "Simulates a flight controller with tunable parameters."

  use GenServer
  @behaviour BB.Parameter.Protocol

  defmodule ParamValue do
    @behaviour BB.Message
    defstruct [:value]
    @schema Spark.Options.new!(value: [type: :any, required: true])
    @impl BB.Message
    def schema, do: @schema
    defimpl BB.Message.Payload do
      def schema(_), do: @for.schema()
    end
  end

  # Simulated FC parameters
  @fc_params %{
    "PITCH_RATE_P" => 0.1,
    "PITCH_RATE_I" => 0.01,
    "PITCH_RATE_D" => 0.005,
    "ROLL_RATE_P" => 0.1,
    "ROLL_RATE_I" => 0.01,
    "ROLL_RATE_D" => 0.005,
    "YAW_RATE_P" => 0.15,
    "THR_HOVER" => 0.5
  }

  def start_link(opts), do: GenServer.start_link(__MODULE__, opts)

  @impl GenServer
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    BB.PubSub.subscribe(robot, [:param])

    {:ok, %{
      robot: robot,
      params: @fc_params,
      subscriptions: MapSet.new()
    }}
  end

  @impl BB.Parameter.Protocol
  def handle_change(_robot, _changed, state), do: {:ok, state}

  @impl BB.Parameter.Protocol
  def list_remote(state) do
    params = Enum.map(state.params, fn {id, value} ->
      %{id: id, value: value, type: :float, doc: nil, path: id_to_path(id)}
    end)
    {:ok, params, state}
  end

  @impl BB.Parameter.Protocol
  def get_remote(param_id, state) do
    case Map.fetch(state.params, param_id) do
      {:ok, value} -> {:ok, value, state}
      :error -> {:error, :not_found, state}
    end
  end

  @impl BB.Parameter.Protocol
  def set_remote(param_id, value, state) do
    if Map.has_key?(state.params, param_id) do
      state = %{state | params: Map.put(state.params, param_id, value)}

      # Notify subscribers
      if MapSet.member?(state.subscriptions, param_id) do
        path = id_to_path(param_id)
        message = BB.Message.new!(ParamValue, :fc, value: value)
        BB.PubSub.publish(state.robot, path, message)
      end

      {:ok, state}
    else
      {:error, :not_found, state}
    end
  end

  @impl BB.Parameter.Protocol
  def subscribe_remote(param_id, state) do
    {:ok, %{state | subscriptions: MapSet.put(state.subscriptions, param_id)}}
  end

  @impl GenServer
  def handle_info({:bb, [:param | _], message}, state) do
    {:ok, new_state} = handle_change(state.robot, message.payload, state)
    {:noreply, new_state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  defp id_to_path(param_id) do
    atoms = param_id |> String.downcase() |> String.split("_") |> Enum.map(&String.to_atom/1)
    [:fc | atoms]
  end
end
```

Use it in your robot:

```elixir
defmodule TestRobot do
  use BB

  parameters do
    param :armed, type: :boolean, default: false

    bridge :fc, MockFCBridge
  end

  topology do
    link :base
  end
end
```

Try it out:

```elixir
iex> {:ok, _} = BB.Supervisor.start_link(TestRobot)

iex> {:ok, params} = BB.Parameter.list_remote(TestRobot, :fc)
iex> Enum.map(params, & &1.id)
["PITCH_RATE_P", "PITCH_RATE_I", "PITCH_RATE_D", "ROLL_RATE_P", ...]

iex> BB.Parameter.get_remote(TestRobot, :fc, "PITCH_RATE_P")
{:ok, 0.1}

iex> BB.Parameter.set_remote(TestRobot, :fc, "PITCH_RATE_P", 0.15)
:ok

iex> BB.Parameter.get_remote(TestRobot, :fc, "PITCH_RATE_P")
{:ok, 0.15}
```

## Summary

Parameter bridges enable:
- **Local → Remote:** Expose BB parameters to external tools
- **Remote → Local:** Access parameters from connected systems
- **Bidirectional sync:** Keep parameters in sync across systems

Key points:
- Bridges implement `BB.Parameter.Protocol`
- Use `init/2` and `handle_change/3` for outbound (local changes)
- Use `list_remote/1`, `get_remote/2`, `set_remote/3` for inbound (remote access)
- Each bridge is supervised independently for fault isolation
- Access remote params via `BB.Parameter.{list,get,set}_remote`

## What's Next?

You've now learned the complete parameter system. You can:
- Define parameters in the DSL
- Read and write them at runtime
- Subscribe to changes via PubSub
- Connect to remote systems via bridges

For reference documentation on all parameter options, see the [DSL Reference](DSL-BB.md).
