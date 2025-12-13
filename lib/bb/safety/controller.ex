# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.Controller do
  @moduledoc """
  Global safety controller that owns arm/disarm state for all robots.

  Part of BB's application supervision tree (not per-robot), so it survives
  robot crashes and maintains safety state. Runs at high scheduler priority.

  Uses two ETS tables:

  1. **Robots table** (protected set) - safety state per robot, writes only via GenServer
  2. **Handlers table** (public bag) - direct writes for registration

  Monitors robot supervisors and cleans up state when they terminate.

  ## Safety States

  - `:disarmed` - Robot is safely disarmed, all disarm callbacks succeeded
  - `:armed` - Robot is armed and ready to operate
  - `:error` - Disarm attempted but one or more callbacks failed; hardware may not be safe

  When in `:error` state, the robot cannot be armed until `force_disarm/1` is called
  to acknowledge the error and reset to `:disarmed`.

  Note: The executing/idle distinction is handled by Runtime as it's not safety-critical.
  """
  use GenServer
  require Logger

  alias BB.{Message, PubSub}
  alias BB.StateMachine.Transition

  @robots_table Module.concat(__MODULE__, Robots)
  @handlers_table Module.concat(__MODULE__, Handlers)

  @type safety_state :: :disarmed | :armed | :error

  @doc false
  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  @doc """
  Check if a robot is armed.

  Fast ETS read - does not go through GenServer.
  """
  @spec armed?(module()) :: boolean()
  def armed?(robot_module) do
    case :ets.lookup(@robots_table, robot_module) do
      [{^robot_module, :armed, _ref}] -> true
      _ -> false
    end
  end

  @doc """
  Get current safety state for a robot.

  Fast ETS read - does not go through GenServer.
  Returns `:armed`, `:disarmed`, or `:error`.
  """
  @spec state(module()) :: safety_state()
  def state(robot_module) do
    case :ets.lookup(@robots_table, robot_module) do
      [{^robot_module, state, _ref}] -> state
      [] -> :disarmed
    end
  end

  @doc """
  Check if a robot is in error state.

  Returns `true` if a disarm operation failed and the robot requires
  manual intervention via `force_disarm/1`.

  Fast ETS read - does not go through GenServer.
  """
  @spec in_error?(module()) :: boolean()
  def in_error?(robot_module) do
    case :ets.lookup(@robots_table, robot_module) do
      [{^robot_module, :error, _ref}] -> true
      _ -> false
    end
  end

  @doc """
  Register a robot when it starts.

  Called by `BB.Supervisor` during robot startup. Sets up monitoring of the
  robot's supervision tree for automatic cleanup on crash.
  """
  @spec register_robot(module()) :: :ok | {:error, term()}
  def register_robot(robot_module) do
    GenServer.call(__MODULE__, {:register_robot, robot_module})
  end

  @doc """
  Register a safety handler (actuator/sensor/controller).

  Called by processes in their `init/1`. The opts should contain all
  hardware-specific parameters needed to call `disarm/1` without GenServer state.

  Writes directly to ETS to avoid blocking on the Controller's mailbox.
  Uses a cast to set up process monitoring for cleanup on handler restart.
  """
  @spec register(module(), keyword()) :: :ok
  def register(module, opts) do
    robot = Keyword.fetch!(opts, :robot)
    path = Keyword.fetch!(opts, :path)
    disarm_opts = Keyword.get(opts, :opts, [])
    pid = self()

    # Direct ETS write - no GenServer call needed
    :ets.insert(@handlers_table, {robot, module, path, disarm_opts, pid})

    # Async monitoring setup - fire and forget
    GenServer.cast(__MODULE__, {:monitor_handler, pid, robot})

    :ok
  end

  @doc """
  Get list of registered handler modules for a robot.

  Used by Runtime to verify all safety handlers have registered on startup.
  """
  @spec registered_handlers(module()) :: [module()]
  def registered_handlers(robot_module) do
    @handlers_table
    |> :ets.lookup(robot_module)
    |> Enum.map(fn {_robot, module, _path, _opts, _pid} -> module end)
  end

  @doc """
  Arm the robot.

  Goes through GenServer to ensure proper state transitions and event publishing.
  Cannot arm if robot is in `:error` state - must call `force_disarm/1` first.
  """
  @spec arm(module()) :: :ok | {:error, :already_armed | :in_error | :not_registered}
  def arm(robot_module), do: GenServer.call(__MODULE__, {:arm, robot_module})

  @doc """
  Disarm the robot.

  Goes through GenServer. Calls all registered `disarm/1` callbacks before
  updating state. If any callback fails, the robot transitions to `:error`
  state instead of `:disarmed`, and this function returns an error with
  details of the failures.

  When in `:error` state, the robot cannot be armed until `force_disarm/1`
  is called to acknowledge the failure and reset to `:disarmed`.
  """
  @spec disarm(module()) ::
          :ok | {:error, :already_disarmed | :not_registered | {:disarm_failed, list()}}
  def disarm(robot_module), do: GenServer.call(__MODULE__, {:disarm, robot_module})

  @doc """
  Force disarm from error state.

  Use this function to acknowledge a failed disarm operation and reset the
  robot to `:disarmed` state. This should only be called after manually
  verifying that hardware is in a safe state.

  **WARNING**: This bypasses safety checks. Only use when you have manually
  verified that all actuators are disabled and the robot is safe.
  """
  @spec force_disarm(module()) :: :ok | {:error, :not_in_error | :not_registered}
  def force_disarm(robot_module), do: GenServer.call(__MODULE__, {:force_disarm, robot_module})

  # --- GenServer Callbacks ---

  @impl GenServer
  def init(_opts) do
    Process.flag(:priority, :high)

    # Robots table (protected): safety state, writes only via GenServer
    # {robot_module, :armed | :disarmed, supervisor_monitor_ref}
    :ets.new(@robots_table, [:named_table, :protected, :set, read_concurrency: true])

    # Handlers table (public bag): direct writes for registration
    # {robot_module, module, path, opts, handler_pid}
    :ets.new(@handlers_table, [:named_table, :public, :bag, read_concurrency: true])

    {:ok, %{}}
  end

  @impl GenServer
  def handle_call({:register_robot, robot_module}, _from, state) do
    # The supervisor is registered directly with the robot_module name
    case Process.whereis(robot_module) do
      pid when is_pid(pid) ->
        ref = Process.monitor(pid)
        :ets.insert(@robots_table, {robot_module, :disarmed, ref})
        {:reply, :ok, state}

      nil ->
        {:reply, {:error, :supervisor_not_found}, state}
    end
  end

  def handle_call({:arm, robot_module}, _from, state) do
    case :ets.lookup(@robots_table, robot_module) do
      [{^robot_module, :disarmed, ref}] ->
        :ets.insert(@robots_table, {robot_module, :armed, ref})
        publish_transition(robot_module, :disarmed, :armed)
        {:reply, :ok, state}

      [{^robot_module, :armed, _}] ->
        {:reply, {:error, :already_armed}, state}

      [{^robot_module, :error, _}] ->
        {:reply, {:error, :in_error}, state}

      [] ->
        {:reply, {:error, :not_registered}, state}
    end
  end

  def handle_call({:disarm, robot_module}, _from, state) do
    case :ets.lookup(@robots_table, robot_module) do
      [{^robot_module, :disarmed, _}] ->
        {:reply, {:error, :already_disarmed}, state}

      [{^robot_module, :error, _}] ->
        {:reply, {:error, :already_in_error}, state}

      [{^robot_module, :armed, ref}] ->
        # Call all registered disarm handlers and collect failures
        case disarm_all_handlers(robot_module) do
          :ok ->
            :ets.insert(@robots_table, {robot_module, :disarmed, ref})
            publish_transition(robot_module, :armed, :disarmed)
            {:reply, :ok, state}

          {:error, failures} ->
            :ets.insert(@robots_table, {robot_module, :error, ref})
            publish_transition(robot_module, :armed, :error)
            {:reply, {:error, {:disarm_failed, failures}}, state}
        end

      [] ->
        {:reply, {:error, :not_registered}, state}
    end
  end

  def handle_call({:force_disarm, robot_module}, _from, state) do
    case :ets.lookup(@robots_table, robot_module) do
      [{^robot_module, :error, ref}] ->
        Logger.warning(
          "Force disarm called for #{inspect(robot_module)} - " <>
            "operator has acknowledged hardware may not be in safe state"
        )

        :ets.insert(@robots_table, {robot_module, :disarmed, ref})
        publish_transition(robot_module, :error, :disarmed)
        {:reply, :ok, state}

      [{^robot_module, _, _}] ->
        {:reply, {:error, :not_in_error}, state}

      [] ->
        {:reply, {:error, :not_registered}, state}
    end
  end

  @impl GenServer
  def handle_cast({:monitor_handler, pid, robot}, state) do
    # Set up monitoring for cleanup on handler restart
    ref = Process.monitor(pid)
    # Store mapping: ref -> {robot, pid} for cleanup lookup
    {:noreply, Map.put(state, ref, {robot, pid})}
  end

  @impl GenServer
  def handle_info({:DOWN, ref, :process, pid, _reason}, state) do
    # Check if it's a robot supervisor
    case :ets.match(@robots_table, {:"$1", :_, ref}) do
      [[robot_module]] ->
        Logger.warning(
          "Robot #{inspect(robot_module)} supervisor crashed, disarming all handlers"
        )

        # Robot crashed - disarm all handlers and clean up
        case disarm_all_handlers(robot_module) do
          :ok ->
            Logger.info(
              "All disarm callbacks succeeded for crashed robot #{inspect(robot_module)}"
            )

          {:error, failures} ->
            Logger.critical(
              "DISARM CALLBACKS FAILED for crashed robot #{inspect(robot_module)}: " <>
                "#{length(failures)} handler(s) failed to disarm - HARDWARE MAY NOT BE SAFE"
            )
        end

        :ets.delete(@robots_table, robot_module)
        :ets.match_delete(@handlers_table, {robot_module, :_, :_, :_, :_})
        {:noreply, state}

      [] ->
        # It's a handler process - look up from state and remove
        case Map.pop(state, ref) do
          {{robot, ^pid}, new_state} ->
            # Delete the specific handler row from bag table
            :ets.match_delete(@handlers_table, {robot, :_, :_, :_, pid})
            {:noreply, new_state}

          {nil, _} ->
            {:noreply, state}
        end
    end
  end

  # --- Private Functions ---

  defp disarm_all_handlers(robot_module) do
    # Bag table: each row is {robot_module, module, path, opts, pid}
    handlers = :ets.lookup(@handlers_table, robot_module)

    failures =
      handlers
      |> Enum.reduce([], fn {_robot, module, path, opts, _pid}, acc ->
        case safe_disarm(module, path, opts) do
          :ok -> acc
          {:error, error} -> [{path, error} | acc]
        end
      end)
      |> Enum.reverse()

    case failures do
      [] -> :ok
      _ -> {:error, failures}
    end
  end

  defp safe_disarm(module, path, opts) do
    case module.disarm(opts) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.error("Disarm failed for #{inspect(path)}: returned {:error, #{inspect(reason)}}")

        {:error, {:returned_error, reason}}
    end
  rescue
    e ->
      Logger.error("Disarm failed for #{inspect(path)}: #{Exception.message(e)}")
      {:error, {:exception, Exception.message(e)}}
  catch
    kind, reason ->
      Logger.error("Disarm failed for #{inspect(path)}: #{inspect({kind, reason})}")
      {:error, {kind, reason}}
  end

  defp publish_transition(robot_module, from, to) do
    message = Message.new!(Transition, :state_machine, from: from, to: to)
    PubSub.publish(robot_module, [:state_machine], message)
  end
end
