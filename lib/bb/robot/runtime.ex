# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Runtime do
  @moduledoc """
  Runtime process for a BB robot.

  Manages the robot's runtime state including:
  - The `BB.Robot` struct (static topology)
  - The `BB.Robot.State` ETS table (dynamic joint state)
  - Robot state machine (disarmed/idle/executing)
  - Command execution lifecycle
  - Sensor telemetry collection (subscribes to `JointState` messages)

  ## Robot States

  The robot progresses through these states:
  - `:disarmed` - Robot is not armed, commands restricted
  - `:idle` - Robot is armed and ready for commands
  - `:executing` - A command is currently executing

  ## State Transitions

  ```
  :disarmed ──arm──→ :idle
  :idle ──execute──→ :executing
  :executing ──complete──→ :idle
  :executing ──disarm──→ :disarmed
  :idle ──disarm──→ :disarmed
  ```

  ## Command Execution

  Commands execute as supervised GenServers. The caller receives the command
  pid and can use `BB.Command.await/2` or `BB.Command.yield/2` to get the
  result. The Runtime monitors the command server and transitions back to
  `:idle` when it completes.
  """

  use GenServer
  require Logger

  alias BB.Command.{Context, Event}
  alias BB.Dsl.{Info, Joint, Link}
  alias BB.Error.Category.Full, as: CategoryFullError
  alias BB.Error.State.Invalid, as: StateInvalidError
  alias BB.Error.State.NotAllowed, as: StateError
  alias BB.{Message, PubSub}
  alias BB.Message.Sensor.JointState
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Parameter.Schema, as: ParameterSchema
  alias BB.Robot.ParamResolver
  alias BB.Robot.State, as: RobotState
  alias BB.Safety.Controller, as: SafetyController
  alias BB.StateMachine.Transition

  alias BB.Robot.CommandInfo

  defstruct [
    :robot_module,
    :robot,
    :robot_state,
    :operational_state,
    :commands,
    :executing_commands,
    :category_counts,
    :category_limits,
    :valid_states,
    :parameter_store,
    :parameter_store_state,
    :simulation_mode,
    # Legacy fields for backwards compatibility during migration
    :current_command_pid,
    :current_command_ref,
    :current_command_name,
    :current_execution_id
  ]

  @type robot_state :: :disarmed | :disarming | :idle | :executing | :error | atom()
  @type simulation_mode :: nil | :kinematic | :external
  @type t :: %__MODULE__{
          robot_module: module(),
          robot: BB.Robot.t(),
          robot_state: RobotState.t(),
          operational_state: atom(),
          commands: %{atom() => BB.Dsl.Command.t()},
          executing_commands: %{reference() => CommandInfo.t()},
          category_counts: %{atom() => non_neg_integer()},
          category_limits: %{atom() => pos_integer()},
          valid_states: [atom()],
          parameter_store: module() | nil,
          parameter_store_state: term() | nil,
          simulation_mode: simulation_mode(),
          # Legacy fields
          current_command_pid: pid() | nil,
          current_command_ref: reference() | nil,
          current_command_name: atom() | nil,
          current_execution_id: reference() | nil
        }

  @doc """
  Starts the runtime for a robot module.
  """
  def start_link({robot_module, opts}) do
    GenServer.start_link(__MODULE__, {robot_module, opts}, name: via(robot_module))
  end

  @doc """
  Returns the via tuple for process registration.
  """
  def via(robot_module) do
    BB.Process.via(robot_module, __MODULE__)
  end

  @doc """
  Get the current robot state machine state.

  Returns `:disarmed` if the robot is not armed (via BB.Safety),
  otherwise returns the internal operational state.

  For backwards compatibility:
  - When `operational_state` is `:idle` but commands are executing, returns `:executing`
  - Custom operational states (e.g., `:recording`) are returned directly

  Reads directly from ETS for fast concurrent access.
  """
  @spec state(module()) :: robot_state()
  def state(robot_module) do
    safety_state = BB.Safety.state(robot_module)

    case safety_state do
      :armed ->
        robot_state = get_robot_state(robot_module)
        internal_state = RobotState.get_robot_state(robot_state)

        # Backwards compatibility: when operational_state is :idle and commands
        # are running, return :executing
        if internal_state == :idle and executing?(robot_module) do
          :executing
        else
          internal_state
        end

      :disarmed ->
        :disarmed

      :disarming ->
        :disarming

      :error ->
        :error
    end
  end

  @doc """
  Get the actual operational state, without backwards compatibility translation.

  Unlike `state/1`, this returns the actual operational state regardless of
  whether commands are executing. Use this when you need to know the true
  operational context (e.g., `:idle`, `:recording`, `:reacting`).

  Reads directly from ETS for fast concurrent access.
  """
  @spec operational_state(module()) :: atom()
  def operational_state(robot_module) do
    robot_state = get_robot_state(robot_module)
    RobotState.get_robot_state(robot_state)
  end

  @doc """
  Check if any command is currently executing.

  Reads directly from ETS for fast concurrent access.
  """
  @spec executing?(module()) :: boolean()
  def executing?(robot_module) do
    GenServer.call(via(robot_module), :any_executing?)
  end

  @doc """
  Check if a specific category has commands executing.
  """
  @spec executing?(module(), atom()) :: boolean()
  def executing?(robot_module, category) do
    GenServer.call(via(robot_module), {:category_executing?, category})
  end

  @doc """
  Get information about all currently executing commands.
  """
  @spec executing_commands(module()) :: [map()]
  def executing_commands(robot_module) do
    GenServer.call(via(robot_module), :executing_commands)
  end

  @doc """
  Get the availability of each command category.

  Returns a map of category names to `{current_count, limit}` tuples.
  """
  @spec category_availability(module()) :: %{atom() => {non_neg_integer(), pos_integer()}}
  def category_availability(robot_module) do
    GenServer.call(via(robot_module), :category_availability)
  end

  @doc """
  Transition the operational state during command execution.

  This is called by `BB.Command.transition_state/2` to change the robot's
  operational state mid-execution. Only the command with the matching
  execution_id can trigger a transition.
  """
  @spec transition_operational_state(module(), reference(), atom()) :: :ok | {:error, term()}
  def transition_operational_state(robot_module, execution_id, target_state) do
    GenServer.call(via(robot_module), {:transition_operational_state, execution_id, target_state})
  end

  @doc """
  Get the simulation mode for a robot.

  Returns `nil` if running in hardware mode, or the simulation mode atom
  (e.g., `:kinematic`, `:external`) if running in simulation.
  """
  @spec simulation_mode(module()) :: simulation_mode()
  def simulation_mode(robot_module) do
    GenServer.call(via(robot_module), :get_simulation_mode)
  end

  @doc """
  Transition the robot to a new state.
  """
  @spec transition(module(), robot_state()) :: {:ok, robot_state()} | {:error, term()}
  def transition(robot_module, new_state) do
    GenServer.call(via(robot_module), {:transition, new_state})
  end

  @doc """
  Check if the robot is in one of the allowed states.

  Reads directly from ETS for fast concurrent access.
  """
  @spec check_allowed(module(), [robot_state()]) :: :ok | {:error, StateError.t()}
  def check_allowed(robot_module, allowed_states) do
    current = state(robot_module)

    if current in allowed_states do
      :ok
    else
      {:error, StateError.exception(current_state: current, allowed_states: allowed_states)}
    end
  end

  @doc """
  Get the robot state (ETS-backed joint positions/velocities).
  """
  @spec get_robot_state(module()) :: RobotState.t()
  def get_robot_state(robot_module) do
    GenServer.call(via(robot_module), :get_robot_state)
  end

  @doc """
  Get the static robot struct (topology).
  """
  @spec get_robot(module()) :: BB.Robot.t()
  def get_robot(robot_module) do
    GenServer.call(via(robot_module), :get_robot)
  end

  @doc """
  Get all joint positions as a map.

  Reads directly from ETS for fast concurrent access. Returns a map of
  joint names to their current positions (in radians for revolute joints,
  metres for prismatic joints).

  Positions are updated automatically by the Runtime when sensors publish
  `JointState` messages.

  ## Examples

      iex> BB.Robot.Runtime.positions(MyRobot)
      %{pan_joint: 0.0, tilt_joint: 0.0}

  """
  @spec positions(module()) :: %{atom() => float()}
  def positions(robot_module) do
    robot_state = get_robot_state(robot_module)
    RobotState.get_all_positions(robot_state)
  end

  @doc """
  Get all joint velocities as a map.

  Reads directly from ETS for fast concurrent access. Returns a map of
  joint names to their current velocities (in rad/s for revolute joints,
  m/s for prismatic joints).

  Velocities are updated automatically by the Runtime when sensors publish
  `JointState` messages.

  ## Examples

      iex> BB.Robot.Runtime.velocities(MyRobot)
      %{pan_joint: 0.0, tilt_joint: 0.0}

  """
  @spec velocities(module()) :: %{atom() => float()}
  def velocities(robot_module) do
    robot_state = get_robot_state(robot_module)
    RobotState.get_all_velocities(robot_state)
  end

  @doc """
  Execute a command with the given goal.

  Returns `{:ok, pid}` where `pid` is the command server process.
  Use `BB.Command.await/2` or `BB.Command.yield/2` to get the result.

  ## Examples

      {:ok, cmd} = Runtime.execute(MyRobot, :navigate, %{target: pose})
      {:ok, result} = BB.Command.await(cmd)

      # Or with timeout
      case BB.Command.yield(cmd, 5000) do
        nil -> still_running()
        {:ok, result} -> handle_result(result)
        {:error, reason} -> handle_error(reason)
      end

  ## Errors

  - `{:error, %StateError{}}` - Robot not in allowed state
  - `{:error, {:unknown_command, name}}` - Command not found
  - Other errors are returned through `BB.Command.await/2`
  """
  @spec execute(module(), atom(), map()) :: {:ok, pid()} | {:error, term()}
  def execute(robot_module, command_name, goal) do
    execution_id = make_ref()

    case GenServer.call(via(robot_module), {:execute, command_name, goal, execution_id}) do
      {:ok, pid} ->
        {:ok, pid}

      {:error, _} = error ->
        error
    end
  end

  @doc """
  Cancel the currently executing command.

  Stops the command server with `:cancelled` reason. Awaiting callers
  will receive the result from the command's `result/1` callback.
  """
  @spec cancel(module()) :: :ok | {:error, :no_execution}
  def cancel(robot_module) do
    GenServer.call(via(robot_module), :cancel)
  end

  @impl GenServer
  def init({robot_module, opts}) do
    # Register robot with the safety controller for arm/disarm state management
    :ok = SafetyController.register_robot(robot_module)

    robot = robot_module.robot()
    {:ok, robot_state} = RobotState.new(robot)

    commands =
      robot_module
      |> Info.commands()
      |> Map.new(&{&1.name, &1})

    # Subscribe to all sensor messages to receive JointState updates
    PubSub.subscribe(robot_module, [:sensor])

    # Initialize parameter store if configured
    {store_module, store_state} = init_parameter_store(robot_module)

    # Get initial operational state and valid states from DSL
    initial_state = Info.initial_state(robot_module)
    valid_states = Info.state_names(robot_module)
    category_limits = Info.category_limits(robot_module)

    # Internal state tracks operational state (not safety state)
    # The armed/disarmed state is owned by SafetyController
    simulation_mode = Keyword.get(opts, :simulation)

    state = %__MODULE__{
      robot_module: robot_module,
      robot: robot,
      robot_state: robot_state,
      operational_state: initial_state,
      commands: commands,
      executing_commands: %{},
      category_counts: Map.new(Map.keys(category_limits), &{&1, 0}),
      category_limits: category_limits,
      valid_states: valid_states,
      parameter_store: store_module,
      parameter_store_state: store_state,
      simulation_mode: simulation_mode,
      # Legacy fields - kept for backwards compatibility
      current_command_pid: nil,
      current_command_ref: nil,
      current_command_name: nil,
      current_execution_id: nil
    }

    # Register DSL-defined parameters (applies defaults)
    register_dsl_parameters(state)

    # Load and apply persisted values (override defaults)
    state = load_persisted_parameters(state)

    # Apply start_link params (override persisted values)
    case apply_startup_params(state, opts) do
      {:ok, state} ->
        # Resolve param refs and subscribe to changes
        state = resolve_and_subscribe_param_refs(state)
        # Set initial operational state in ETS
        state = set_robot_machine_state(state, initial_state)
        {:ok, state, {:continue, :schedule_safety_verification}}

      {:error, reason} ->
        {:stop, reason}
    end
  end

  defp resolve_and_subscribe_param_refs(state) do
    robot = state.robot

    if map_size(robot.param_subscriptions) > 0 do
      # Resolve all param refs using current parameter values
      resolved_robot = ParamResolver.resolve_all(robot, state.robot_state)

      # Subscribe to parameter changes for all referenced parameters
      for param_path <- Map.keys(robot.param_subscriptions) do
        PubSub.subscribe(state.robot_module, [:param | param_path])
      end

      %{state | robot: resolved_robot}
    else
      state
    end
  end

  @impl GenServer
  def handle_continue(:schedule_safety_verification, state) do
    # Allow time for child processes to start and register
    Process.send_after(self(), :verify_safety_registrations, 1000)
    {:noreply, state}
  end

  @impl GenServer
  def handle_call({:transition, new_state}, _from, state) do
    old_state = state.operational_state

    if old_state != new_state do
      state = set_robot_machine_state(state, new_state)
      publish_transition(state, old_state, new_state)
      {:reply, {:ok, new_state}, state}
    else
      {:reply, {:ok, new_state}, state}
    end
  end

  def handle_call(:get_robot_state, _from, state) do
    {:reply, state.robot_state, state}
  end

  def handle_call(:get_robot, _from, state) do
    {:reply, state.robot, state}
  end

  def handle_call(:get_simulation_mode, _from, state) do
    {:reply, state.simulation_mode, state}
  end

  def handle_call(:any_executing?, _from, state) do
    {:reply, map_size(state.executing_commands) > 0, state}
  end

  def handle_call({:category_executing?, category}, _from, state) do
    count = Map.get(state.category_counts, category, 0)
    {:reply, count > 0, state}
  end

  def handle_call(:executing_commands, _from, state) do
    commands =
      state.executing_commands
      |> Map.values()
      |> Enum.map(fn %CommandInfo{} = info ->
        %{
          name: info.name,
          execution_id: info.ref,
          pid: info.pid,
          category: info.category,
          started_at: info.started_at
        }
      end)

    {:reply, commands, state}
  end

  def handle_call(:category_availability, _from, state) do
    availability =
      Map.new(state.category_limits, fn {category, limit} ->
        current = Map.get(state.category_counts, category, 0)
        {category, {current, limit}}
      end)

    {:reply, availability, state}
  end

  def handle_call({:transition_operational_state, execution_id, target_state}, _from, state) do
    cond do
      not Map.has_key?(state.executing_commands, execution_id) ->
        {:reply, {:error, :not_executing}, state}

      target_state not in state.valid_states ->
        {:reply,
         {:error,
          StateInvalidError.exception(state: target_state, valid_states: state.valid_states)},
         state}

      true ->
        old_state = state.operational_state
        state = set_robot_machine_state(state, target_state)
        state = %{state | operational_state: target_state}

        if old_state != target_state do
          publish_transition(state, old_state, target_state)
        end

        {:reply, :ok, state}
    end
  end

  def handle_call({:execute, command_name, goal, execution_id}, _from, state) do
    case Map.fetch(state.commands, command_name) do
      {:ok, command} ->
        handle_execute_command(command, goal, execution_id, state)

      :error ->
        {:reply, {:error, {:unknown_command, command_name}}, state}
    end
  end

  def handle_call(:cancel, _from, %{current_command_pid: nil} = state) do
    {:reply, {:error, :no_execution}, state}
  end

  def handle_call(:cancel, _from, state) do
    # Stop the command server - it will notify us via {:command_complete, ...} cast
    BB.Command.cancel(state.current_command_pid)
    {:reply, :ok, state}
  end

  def handle_call({:command_complete, _execution_id, _next_state}, _from, state) do
    # This is now handled via cast, but keep for backwards compatibility
    {:reply, :ok, state}
  end

  # Parameter handling

  def handle_call({:set_parameter, path, value}, _from, state) do
    case validate_and_set_parameter(state, path, value) do
      {:ok, old_value} ->
        save_to_store(state, path, value)
        publish_parameter_change(state.robot_module, path, old_value, value, :local)
        {:reply, :ok, state}

      {:error, _} = error ->
        {:reply, error, state}
    end
  end

  def handle_call({:set_parameters, params}, _from, state) do
    case validate_all_parameters(state, params) do
      :ok ->
        # All valid - apply changes, save, and notify
        Enum.each(params, fn {path, value} ->
          old_value = get_current_param_value(state, path)
          RobotState.set_parameter(state.robot_state, path, value)
          save_to_store(state, path, value)
          publish_parameter_change(state.robot_module, path, old_value, value, :local)
        end)

        {:reply, :ok, state}

      {:error, _} = error ->
        {:reply, error, state}
    end
  end

  def handle_call({:register_parameters, path, component_module}, _from, state) do
    case register_component_parameters(state, path, component_module) do
      :ok -> {:reply, :ok, state}
      {:error, _} = error -> {:reply, error, state}
    end
  end

  @impl GenServer
  def handle_cast({:command_complete, execution_id, result}, state) do
    if Map.has_key?(state.executing_commands, execution_id) do
      handle_command_completion(state, execution_id, result)
    else
      # Stale completion - ignore
      {:noreply, state}
    end
  end

  def handle_cast({:command_crashed, execution_id, error}, state) do
    case Map.get(state.executing_commands, execution_id) do
      nil ->
        {:noreply, state}

      command_info ->
        Logger.error("Command #{inspect(command_info.name)} crashed: #{inspect(error)}")
        handle_command_completion(state, execution_id, {:error, error})
    end
  end

  defp handle_command_completion(state, execution_id, result) do
    command_info = Map.get(state.executing_commands, execution_id)
    old_state = state.operational_state

    demonitor_command(command_info)

    next_state = extract_next_state(result, old_state)
    publish_command_result(state.robot_module, command_info, execution_id, result)

    state = remove_command_from_tracking(state, execution_id, command_info.category)
    was_last_command = map_size(state.executing_commands) == 0

    state = handle_completion_transitions(state, old_state, next_state, was_last_command)

    {:noreply, state}
  end

  defp demonitor_command(command_info) do
    if command_info && command_info.ref do
      Process.demonitor(command_info.ref, [:flush])
    end
  end

  defp publish_command_result(robot_module, command_info, execution_id, result) do
    path = [:command, command_info.name, execution_id]

    case result do
      {:ok, value} ->
        publish_command_event(robot_module, path, :succeeded, %{result: value})

      {:ok, value, _opts} ->
        publish_command_event(robot_module, path, :succeeded, %{result: value})

      {:error, reason} ->
        publish_command_event(robot_module, path, :failed, %{reason: reason})
    end
  end

  defp remove_command_from_tracking(state, execution_id, category) do
    new_executing = Map.delete(state.executing_commands, execution_id)
    new_counts = Map.update(state.category_counts, category, 0, &max(&1 - 1, 0))

    {legacy_pid, legacy_ref, legacy_name, legacy_id} =
      if state.current_execution_id == execution_id do
        {nil, nil, nil, nil}
      else
        {state.current_command_pid, state.current_command_ref, state.current_command_name,
         state.current_execution_id}
      end

    %{
      state
      | executing_commands: new_executing,
        category_counts: new_counts,
        current_command_pid: legacy_pid,
        current_command_ref: legacy_ref,
        current_command_name: legacy_name,
        current_execution_id: legacy_id
    }
  end

  defp handle_completion_transitions(state, old_state, next_state, was_last_command) do
    cond do
      old_state != next_state ->
        state = set_robot_machine_state(state, next_state)

        # Backwards compat: from :idle state, show :executing -> next_state
        if was_last_command and old_state == :idle do
          publish_transition(state, :executing, next_state)
        else
          publish_transition(state, old_state, next_state)
        end

        state

      was_last_command and old_state == :idle ->
        # No state change, but publish :executing -> :idle for backwards compat
        publish_transition(state, :executing, :idle)
        state

      true ->
        state
    end
  end

  defp extract_next_state({:ok, _value, opts}, current_state) when is_list(opts) do
    Keyword.get(opts, :next_state, current_state)
  end

  defp extract_next_state(_, current_state), do: current_state

  defp find_command_by_ref(executing_commands, ref) do
    Enum.find_value(executing_commands, fn {execution_id, command_info} ->
      if command_info.ref == ref do
        {execution_id, command_info}
      end
    end)
  end

  @impl GenServer
  def handle_info({:DOWN, ref, :process, _pid, reason}, state) do
    # Find the command with this monitor ref
    case find_command_by_ref(state.executing_commands, ref) do
      {execution_id, command_info} ->
        Logger.warning("Command #{inspect(command_info.name)} process died: #{inspect(reason)}")

        # Treat crash as completion with error result
        handle_command_completion(state, execution_id, {:error, {:crashed, reason}})

      nil ->
        {:noreply, state}
    end
  end

  def handle_info({:bb, _path, %Message{payload: %JointState{} = joint_state}}, state) do
    update_joint_state(state.robot_state, joint_state)
    {:noreply, state}
  end

  def handle_info(
        {:bb, [:param | param_path], %Message{payload: %ParameterChanged{new_value: new_value}}},
        state
      ) do
    if Map.has_key?(state.robot.param_subscriptions, param_path) do
      robot =
        ParamResolver.update_for_param(
          state.robot,
          param_path,
          new_value,
          state.robot_state
        )

      {:noreply, %{state | robot: robot}}
    else
      {:noreply, state}
    end
  end

  def handle_info(:verify_safety_registrations, state) do
    verify_safety_registrations(state)
    {:noreply, state}
  end

  def handle_info(_msg, state) do
    {:noreply, state}
  end

  @impl GenServer
  def terminate(_reason, state) do
    # Close parameter store
    close_parameter_store(state)

    if state.robot_state do
      RobotState.delete(state.robot_state)
    end

    :ok
  end

  defp handle_execute_command(command, goal, execution_id, state) do
    category = command.category || :default

    with :ok <- check_state_allowed(command, state),
         {:ok, state} <- check_category_or_cancel(command, category, state) do
      {:ok, pid} = spawn_command_server(state, command, goal, execution_id)
      monitor_ref = Process.monitor(pid)

      # Publish command started event
      path = [:command, command.name, execution_id]
      publish_command_event(state.robot_module, path, :started, %{goal: goal})

      # Track the command
      command_info = %CommandInfo{
        name: command.name,
        pid: pid,
        ref: monitor_ref,
        category: category,
        started_at: DateTime.utc_now()
      }

      new_state = %{
        state
        | executing_commands: Map.put(state.executing_commands, execution_id, command_info),
          category_counts: Map.update(state.category_counts, category, 1, &(&1 + 1)),
          # Legacy fields for backwards compatibility
          current_command_pid: pid,
          current_command_ref: monitor_ref,
          current_command_name: command.name,
          current_execution_id: execution_id
      }

      # Backwards compatibility: publish :idle -> :executing transition when first command starts
      # This maintains the old PubSub contract where state would transition to :executing
      if map_size(state.executing_commands) == 0 and state.operational_state == :idle do
        publish_transition(new_state, :idle, :executing)
      end

      {:reply, {:ok, pid}, new_state}
    else
      {:error, _} = err ->
        {:reply, err, state}
    end
  end

  defp check_category_or_cancel(command, category, state) do
    # First, cancel commands in categories specified by the command's cancel option
    state =
      case command.cancel do
        [] -> state
        categories -> cancel_commands_in_categories(state, categories)
      end

    # Now check if there's capacity in the command's own category
    current = Map.get(state.category_counts, category, 0)
    limit = Map.get(state.category_limits, category, 1)

    if current < limit do
      {:ok, state}
    else
      {:error, CategoryFullError.exception(category: category, limit: limit, current: current)}
    end
  end

  defp cancel_commands_in_categories(state, categories) do
    # Find all commands in the specified categories and terminate them
    {to_terminate, to_keep} =
      Enum.split_with(state.executing_commands, fn {_id, cmd} ->
        cmd.category in categories
      end)

    # Nothing to cancel
    if to_terminate == [] do
      state
    else
      # Terminate each command
      Enum.each(to_terminate, fn {_id, cmd} ->
        BB.Command.cancel(cmd.pid)
        Process.demonitor(cmd.ref, [:flush])
      end)

      # Calculate new category counts
      terminated_pids = MapSet.new(to_terminate, fn {_, c} -> c.pid end)
      terminated_refs = MapSet.new(to_terminate, fn {_, c} -> c.ref end)
      terminated_ids = MapSet.new(to_terminate, fn {id, _} -> id end)

      new_category_counts =
        Enum.reduce(to_terminate, state.category_counts, fn {_id, cmd}, counts ->
          Map.update(counts, cmd.category, 0, &max(&1 - 1, 0))
        end)

      # Update state
      %{
        state
        | executing_commands: Map.new(to_keep),
          category_counts: new_category_counts,
          # Clear legacy fields if the tracked command was terminated
          current_command_pid:
            if(state.current_command_pid in terminated_pids,
              do: nil,
              else: state.current_command_pid
            ),
          current_command_ref:
            if(state.current_command_ref in terminated_refs,
              do: nil,
              else: state.current_command_ref
            ),
          current_command_name:
            if(state.current_execution_id in terminated_ids,
              do: nil,
              else: state.current_command_name
            ),
          current_execution_id:
            if(state.current_execution_id in terminated_ids,
              do: nil,
              else: state.current_execution_id
            )
      }
    end
  end

  defp spawn_command_server(state, command, goal, execution_id) do
    robot_module = state.robot_module
    robot = state.robot
    robot_state = state.robot_state

    # Build context
    context = %Context{
      robot_module: robot_module,
      robot: robot,
      robot_state: robot_state,
      execution_id: execution_id
    }

    # Extract handler module and options from child_spec format
    {handler_module, handler_opts} = normalize_handler(command.handler)

    child_spec = %{
      id: execution_id,
      start:
        {BB.Command.Server, :start_link,
         [
           [
             callback_module: handler_module,
             context: context,
             goal: goal,
             execution_id: execution_id,
             runtime_pid: self(),
             timeout: command.timeout,
             options: handler_opts
           ]
         ]},
      restart: :temporary
    }

    DynamicSupervisor.start_child(command_supervisor_name(robot_module), child_spec)
  end

  defp normalize_handler({module, opts}) when is_atom(module) and is_list(opts) do
    {module, opts}
  end

  defp normalize_handler(module) when is_atom(module) do
    {module, []}
  end

  defp command_supervisor_name(robot_module) do
    BB.Process.via(robot_module, BB.CommandSupervisor)
  end

  defp publish_command_event(robot_module, path, status, data) do
    message = Message.new!(Event, :command, status: status, data: data)
    PubSub.publish(robot_module, path, message)
  end

  defp check_state_allowed(command, state) do
    case BB.Safety.state(state.robot_module) do
      :error -> {:error, :safety_error}
      :disarming -> {:error, :disarming}
      :armed -> check_operational_state(command, state)
      :disarmed -> check_disarmed_state(command)
    end
  end

  defp check_operational_state(command, state) do
    current_state = state.operational_state
    allowed_states = command.allowed_states

    if current_state in allowed_states do
      :ok
    else
      {:error, StateError.exception(current_state: current_state, allowed_states: allowed_states)}
    end
  end

  defp check_disarmed_state(command) do
    if :disarmed in command.allowed_states do
      :ok
    else
      {:error,
       StateError.exception(current_state: :disarmed, allowed_states: command.allowed_states)}
    end
  end

  defp publish_transition(state, from, to) do
    message = Message.new!(Transition, :state_machine, from: from, to: to)
    PubSub.publish(state.robot_module, [:state_machine], message)
  end

  defp set_robot_machine_state(state, new_robot_state) do
    RobotState.set_robot_state(state.robot_state, new_robot_state)
    %{state | operational_state: new_robot_state}
  end

  defp update_joint_state(robot_state, %JointState{} = joint_state) do
    names = joint_state.names || []
    positions = joint_state.positions || []
    velocities = joint_state.velocities || []

    # Update positions
    names
    |> Enum.zip(positions)
    |> Enum.each(fn {name, position} ->
      RobotState.set_joint_position(robot_state, name, position)
    end)

    # Update velocities
    names
    |> Enum.zip(velocities)
    |> Enum.each(fn {name, velocity} ->
      RobotState.set_joint_velocity(robot_state, name, velocity)
    end)
  end

  # Parameter helpers

  defp validate_and_set_parameter(state, path, value) do
    old_value = get_current_param_value(state, path)

    case validate_parameter(state, path, value) do
      :ok ->
        RobotState.set_parameter(state.robot_state, path, value)
        {:ok, old_value}

      {:error, _} = error ->
        error
    end
  end

  defp validate_all_parameters(state, params) do
    errors =
      params
      |> Enum.map(fn {path, value} ->
        case validate_parameter(state, path, value) do
          :ok -> nil
          {:error, reason} -> {path, reason}
        end
      end)
      |> Enum.reject(&is_nil/1)

    case errors do
      [] -> :ok
      errors -> {:error, errors}
    end
  end

  defp validate_parameter(state, path, value) do
    case RobotState.find_schema_for_parameter(state.robot_state, path) do
      {:ok, schema_path, schema} ->
        # Extract the parameter name from the path
        param_name =
          path
          |> Enum.drop(length(schema_path))
          |> List.first()

        validate_against_schema(schema, param_name, value)

      {:error, :not_found} ->
        {:error, {:unregistered_parameter, path}}
    end
  end

  defp validate_against_schema(%Spark.Options{schema: schema_opts}, param_name, value) do
    case Keyword.fetch(schema_opts, param_name) do
      {:ok, param_opts} ->
        # Build a mini-schema for just this parameter
        mini_schema = Spark.Options.new!([{param_name, param_opts}])

        case Spark.Options.validate([{param_name, value}], mini_schema) do
          {:ok, _} -> :ok
          {:error, error} -> {:error, error}
        end

      :error ->
        {:error, {:unknown_parameter, param_name}}
    end
  end

  defp get_current_param_value(state, path) do
    case RobotState.get_parameter(state.robot_state, path) do
      {:ok, value} -> value
      {:error, :not_found} -> nil
    end
  end

  defp register_component_parameters(state, path, component_module) do
    if BB.Parameter.implements?(component_module) do
      schema = component_module.param_schema()
      RobotState.register_parameter_schema(state.robot_state, path, schema)

      # Initialise parameters with defaults from schema
      initialise_defaults_from_schema(state, path, schema)

      :ok
    else
      {:error, {:not_a_parameter_component, component_module}}
    end
  end

  defp initialise_defaults_from_schema(state, base_path, %Spark.Options{schema: schema_opts}) do
    Enum.each(schema_opts, fn {param_name, param_opts} ->
      case Keyword.fetch(param_opts, :default) do
        {:ok, default} ->
          full_path = base_path ++ [param_name]
          RobotState.set_parameter(state.robot_state, full_path, default)
          publish_parameter_change(state.robot_module, full_path, nil, default, :init)

        :error ->
          :ok
      end
    end)
  end

  defp publish_parameter_change(robot_module, path, old_value, new_value, source) do
    message =
      Message.new!(ParameterChanged, :parameter,
        path: path,
        old_value: old_value,
        new_value: new_value,
        source: source
      )

    PubSub.publish(robot_module, [:param | path], message)
  end

  # Parameter store helpers

  defp init_parameter_store(robot_module) do
    case Info.settings(robot_module).parameter_store do
      nil ->
        {nil, nil}

      store_module when is_atom(store_module) ->
        init_store(store_module, robot_module, [])

      {store_module, opts} when is_atom(store_module) and is_list(opts) ->
        init_store(store_module, robot_module, opts)
    end
  end

  defp init_store(store_module, robot_module, opts) do
    case store_module.init(robot_module, opts) do
      {:ok, store_state} ->
        {store_module, store_state}

      {:error, reason} ->
        require Logger

        Logger.warning(
          "Failed to initialize parameter store #{inspect(store_module)}: #{inspect(reason)}"
        )

        {nil, nil}
    end
  end

  defp load_persisted_parameters(%{parameter_store: nil} = state), do: state

  defp load_persisted_parameters(
         %{parameter_store: store, parameter_store_state: store_state} = state
       ) do
    case store.load(store_state) do
      {:ok, parameters} ->
        Enum.each(parameters, &apply_persisted_value(state, &1))
        state

      {:error, reason} ->
        require Logger
        Logger.warning("Failed to load persisted parameters: #{inspect(reason)}")
        state
    end
  end

  defp apply_persisted_value(state, {path, value}) do
    case RobotState.get_parameter(state.robot_state, path) do
      {:ok, _current} ->
        RobotState.set_parameter(state.robot_state, path, value)
        publish_parameter_change(state.robot_module, path, nil, value, :persisted)

      {:error, :not_found} ->
        :ok
    end
  end

  defp save_to_store(%{parameter_store: nil}, _path, _value), do: :ok

  defp save_to_store(%{parameter_store: store, parameter_store_state: store_state}, path, value) do
    case store.save(store_state, path, value) do
      :ok ->
        :ok

      {:error, reason} ->
        require Logger
        Logger.warning("Failed to persist parameter #{inspect(path)}: #{inspect(reason)}")
        :ok
    end
  end

  defp close_parameter_store(%{parameter_store: nil}), do: :ok

  defp close_parameter_store(%{parameter_store: store, parameter_store_state: store_state}) do
    store.close(store_state)
  end

  defp register_dsl_parameters(state) do
    robot_module = state.robot_module

    if function_exported?(robot_module, :__bb_parameter_schema__, 0) do
      schema_list = robot_module.__bb_parameter_schema__()
      defaults = robot_module.__bb_default_parameters__()

      schema_list
      |> Enum.group_by(fn {path, _opts} -> Enum.take(path, length(path) - 1) end)
      |> Enum.each(&register_schema_group(state.robot_state, &1))

      Enum.each(defaults, &apply_default_value(state, &1))
    end
  end

  defp register_schema_group(robot_state, {prefix_path, params}) do
    schema_opts = Enum.map(params, fn {path, opts} -> {List.last(path), opts} end)
    schema = Spark.Options.new!(schema_opts)
    RobotState.register_parameter_schema(robot_state, prefix_path, schema)
  end

  defp apply_default_value(state, {path, value}) do
    RobotState.set_parameter(state.robot_state, path, value)
    publish_parameter_change(state.robot_module, path, nil, value, :init)
  end

  defp apply_startup_params(state, opts) do
    case Keyword.fetch(opts, :params) do
      {:ok, params} when is_list(params) ->
        validate_and_apply_startup_params(state, params)

      :error ->
        {:ok, state}
    end
  end

  defp validate_and_apply_startup_params(state, params) do
    robot_module = state.robot_module

    if function_exported?(robot_module, :__bb_parameter_schema__, 0) do
      schema = ParameterSchema.build_nested_schema(robot_module.__bb_parameter_schema__())

      with {:ok, validated} <- Spark.Options.validate(params, schema) do
        apply_validated_startup_params(state, validated)
      end
    else
      {:ok, state}
    end
  end

  defp apply_validated_startup_params(state, validated) do
    validated
    |> ParameterSchema.flatten_params()
    |> Enum.each(fn {path, value} ->
      RobotState.set_parameter(state.robot_state, path, value)
      publish_parameter_change(state.robot_module, path, nil, value, :startup)
    end)

    {:ok, state}
  end

  # Safety registration verification

  defp verify_safety_registrations(state) do
    robot_module = state.robot_module
    expected = find_safety_implementers(robot_module)
    registered = SafetyController.registered_handlers(robot_module)

    missing = expected -- registered

    if missing != [] do
      Logger.warning(
        "Safety verification for #{inspect(robot_module)}: " <>
          "#{length(missing)} module(s) implement BB.Safety but have not registered: " <>
          inspect(missing)
      )
    end
  end

  defp find_safety_implementers(robot_module) do
    # Collect modules from robot-level sensors
    robot_sensors =
      robot_module
      |> Info.sensors()
      |> Enum.map(&extract_module(&1.child_spec))
      |> Enum.filter(&implements_safety?/1)

    # Collect modules from controllers
    controllers =
      robot_module
      |> Info.controllers()
      |> Enum.map(&extract_module(&1.child_spec))
      |> Enum.filter(&implements_safety?/1)

    # Collect modules from topology (link sensors, joint sensors/actuators)
    topology_modules = find_topology_safety_implementers(robot_module)

    Enum.uniq(robot_sensors ++ controllers ++ topology_modules)
  end

  defp find_topology_safety_implementers(robot_module) do
    robot_module
    |> Info.topology()
    |> collect_from_topology([])
  end

  defp collect_from_topology([], acc), do: acc

  defp collect_from_topology([entity | rest], acc) do
    acc = collect_entity_modules(entity, acc)
    collect_from_topology(rest, acc)
  end

  defp collect_entity_modules(%Link{sensors: sensors, joints: joints}, acc) do
    sensor_modules =
      sensors
      |> Enum.map(&extract_module(&1.child_spec))
      |> Enum.filter(&implements_safety?/1)

    acc = acc ++ sensor_modules
    collect_from_topology(joints, acc)
  end

  defp collect_entity_modules(
         %Joint{sensors: sensors, actuators: actuators, link: link},
         acc
       ) do
    sensor_modules =
      sensors
      |> Enum.map(&extract_module(&1.child_spec))
      |> Enum.filter(&implements_safety?/1)

    actuator_modules =
      actuators
      |> Enum.map(&extract_module(&1.child_spec))
      |> Enum.filter(&implements_safety?/1)

    acc = acc ++ sensor_modules ++ actuator_modules

    if link do
      collect_entity_modules(link, acc)
    else
      acc
    end
  end

  defp collect_entity_modules(_other, acc), do: acc

  defp extract_module({module, _opts}) when is_atom(module), do: module
  defp extract_module(module) when is_atom(module), do: module

  defp implements_safety?(module) do
    Spark.implements_behaviour?(module, BB.Safety)
  end
end
