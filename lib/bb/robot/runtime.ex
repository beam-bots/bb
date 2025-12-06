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

  Commands execute in supervised tasks. The caller receives a `Task.t()` and
  can use `Task.await/2` or `Task.yield/2` to get the result. The Runtime
  monitors the task and transitions back to `:idle` when it completes.
  """

  use GenServer

  alias BB.Command.{Context, Event}
  alias BB.Dsl.Info
  alias BB.{Message, PubSub}
  alias BB.Message.Sensor.JointState
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot.State, as: RobotState
  alias BB.StateMachine.Transition

  defmodule StateError do
    @moduledoc """
    Error raised when a command is not allowed in the current robot state.
    """
    defexception [:current_state, :allowed_states, :message]

    @type t :: %__MODULE__{
            current_state: atom(),
            allowed_states: [atom()],
            message: String.t()
          }

    @impl true
    def exception(opts) do
      current = Keyword.fetch!(opts, :current_state)
      allowed = Keyword.fetch!(opts, :allowed_states)

      msg =
        Keyword.get_lazy(opts, :message, fn ->
          "Robot is in state #{inspect(current)}, but command requires one of #{inspect(allowed)}"
        end)

      %__MODULE__{current_state: current, allowed_states: allowed, message: msg}
    end
  end

  defstruct [
    :robot_module,
    :robot,
    :robot_state,
    :state,
    :commands,
    :current_task,
    :current_task_ref,
    :current_command_name,
    :current_execution_id,
    :parameter_store,
    :parameter_store_state
  ]

  @type robot_state :: :disarmed | :idle | :executing
  @type t :: %__MODULE__{
          robot_module: module(),
          robot: BB.Robot.t(),
          robot_state: RobotState.t(),
          state: robot_state(),
          commands: %{atom() => BB.Dsl.Command.t()},
          current_task: Task.t() | nil,
          current_task_ref: reference() | nil,
          current_command_name: atom() | nil,
          current_execution_id: reference() | nil,
          parameter_store: module() | nil,
          parameter_store_state: term() | nil
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

  Reads directly from ETS for fast concurrent access.
  """
  @spec state(module()) :: robot_state()
  def state(robot_module) do
    robot_state = get_robot_state(robot_module)
    RobotState.get_robot_state(robot_state)
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

  Returns `{:ok, task}` where `task` can be awaited for the result.
  The task handles subscription to command events internally.

  ## Examples

      {:ok, task} = Runtime.execute(MyRobot, :navigate, %{target: pose})
      {:ok, result} = Task.await(task)

      # Or with timeout
      case Task.yield(task, 5000) || Task.shutdown(task) do
        {:ok, result} -> handle_result(result)
        nil -> handle_timeout()
      end

  ## Errors

  Errors are returned through the awaited task result:
  - `{:error, %StateError{}}` - Robot not in allowed state
  - `{:error, {:unknown_command, name}}` - Command not found
  - `{:error, reason}` - Command handler returned an error
  """
  @spec execute(module(), atom(), map()) :: {:ok, Task.t()}
  def execute(robot_module, command_name, goal) do
    execution_id = make_ref()

    # Spawn task owned by the caller so they can await it
    task =
      Task.async(fn ->
        path = [:command, command_name, execution_id]
        PubSub.subscribe(robot_module, path)

        try do
          case GenServer.call(via(robot_module), {:execute, command_name, goal, execution_id}) do
            :ok ->
              wait_for_completion(path)

            {:error, _} = error ->
              error
          end
        after
          PubSub.unsubscribe(robot_module, path)
        end
      end)

    {:ok, task}
  end

  defp wait_for_completion(path) do
    receive do
      {:bb, ^path, %Message{payload: %Event{status: :succeeded, data: %{result: result}}}} ->
        {:ok, result}

      {:bb, ^path, %Message{payload: %Event{status: :failed, data: %{reason: reason}}}} ->
        {:error, reason}

      {:bb, ^path, %Message{payload: %Event{status: :cancelled}}} ->
        {:error, :cancelled}
    end
  end

  @doc """
  Cancel the currently executing command.

  Terminates the command task immediately. The caller will receive an exit
  when awaiting the task.
  """
  @spec cancel(module()) :: :ok | {:error, :no_execution}
  def cancel(robot_module) do
    GenServer.call(via(robot_module), :cancel)
  end

  @impl GenServer
  def init({robot_module, _opts}) do
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

    state = %__MODULE__{
      robot_module: robot_module,
      robot: robot,
      robot_state: robot_state,
      state: :disarmed,
      commands: commands,
      current_task: nil,
      current_task_ref: nil,
      current_command_name: nil,
      current_execution_id: nil,
      parameter_store: store_module,
      parameter_store_state: store_state
    }

    # Register DSL-defined parameters (applies defaults)
    register_dsl_parameters(state)

    # Load and apply persisted values (override defaults)
    state = load_persisted_parameters(state)

    {:ok, state}
  end

  @impl GenServer
  def handle_call({:transition, new_state}, _from, state) do
    old_state = state.state

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

  def handle_call({:execute, command_name, goal, execution_id}, _from, state) do
    case Map.fetch(state.commands, command_name) do
      {:ok, command} ->
        handle_execute_command(command, goal, execution_id, state)

      :error ->
        {:reply, {:error, {:unknown_command, command_name}}, state}
    end
  end

  def handle_call(:cancel, _from, %{current_task: nil} = state) do
    {:reply, {:error, :no_execution}, state}
  end

  def handle_call(:cancel, _from, state) do
    old_state = state.state
    state = terminate_current_task(state)
    state = %{state | state: :idle}
    state = set_robot_machine_state(state, :idle)

    if old_state != :idle do
      publish_transition(state, old_state, :idle)
    end

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
  def handle_info(
        {:command_task_done, execution_id, next_state},
        %{current_execution_id: execution_id} = state
      ) do
    # Task completed normally - handle state transition
    old_state = state.state

    # Demonitor and flush any pending :DOWN message
    if state.current_task_ref do
      Process.demonitor(state.current_task_ref, [:flush])
    end

    state = %{
      state
      | state: next_state,
        current_task: nil,
        current_task_ref: nil,
        current_command_name: nil,
        current_execution_id: nil
    }

    state = set_robot_machine_state(state, next_state)

    if old_state != next_state do
      publish_transition(state, old_state, next_state)
    end

    {:noreply, state}
  end

  def handle_info({:DOWN, ref, :process, _pid, _reason}, %{current_task_ref: ref} = state) do
    # Task crashed before sending command_task_done - fall back to :idle
    old_state = state.state

    state = %{
      state
      | state: :idle,
        current_task: nil,
        current_task_ref: nil,
        current_command_name: nil,
        current_execution_id: nil
    }

    state = set_robot_machine_state(state, :idle)

    if old_state != :idle do
      publish_transition(state, old_state, :idle)
    end

    {:noreply, state}
  end

  def handle_info({:bb, _path, %Message{payload: %JointState{} = joint_state}}, state) do
    update_joint_state(state.robot_state, joint_state)
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
    with :ok <- check_state_allowed(command, state),
         {:ok, state} <- maybe_preempt(command, state) do
      task = spawn_command_task(state, command, goal, execution_id)
      task_monitor_ref = Process.monitor(task.pid)

      old_state = state.state

      new_state = %{
        state
        | state: :executing,
          current_task: task,
          current_task_ref: task_monitor_ref,
          current_command_name: command.name,
          current_execution_id: execution_id
      }

      new_state = set_robot_machine_state(new_state, :executing)

      if old_state != :executing do
        publish_transition(new_state, old_state, :executing)
      end

      {:reply, :ok, new_state}
    else
      {:error, _} = err ->
        {:reply, err, state}
    end
  end

  defp spawn_command_task(state, command, goal, execution_id) do
    robot_module = state.robot_module
    robot = state.robot
    robot_state = state.robot_state
    runtime_pid = self()
    path = [:command, command.name, execution_id]

    Task.Supervisor.async_nolink(
      task_supervisor_name(robot_module),
      fn ->
        # Build context
        context = %Context{
          robot_module: robot_module,
          robot: robot,
          robot_state: robot_state,
          execution_id: execution_id
        }

        # Broadcast command started
        publish_command_event(robot_module, path, :started, %{goal: goal})

        # Execute handler and parse result
        raw_result = command.handler.handle_command(goal, context)
        {result, next_state} = parse_handler_result(raw_result)

        # Broadcast result
        case result do
          {:ok, value} ->
            publish_command_event(robot_module, path, :succeeded, %{result: value})

          {:error, reason} ->
            publish_command_event(robot_module, path, :failed, %{reason: reason})
        end

        # Notify runtime with next state before returning
        send(runtime_pid, {:command_task_done, execution_id, next_state})

        result
      end
    )
  end

  defp parse_handler_result({:ok, value, opts}) when is_list(opts) do
    next_state = Keyword.get(opts, :next_state, :idle)
    {{:ok, value}, next_state}
  end

  defp parse_handler_result({:ok, value}) do
    {{:ok, value}, :idle}
  end

  defp parse_handler_result({:error, reason}) do
    {{:error, reason}, :idle}
  end

  defp task_supervisor_name(robot_module) do
    BB.Process.via(robot_module, BB.TaskSupervisor)
  end

  defp publish_command_event(robot_module, path, status, data) do
    message = Message.new!(Event, :command, status: status, data: data)
    PubSub.publish(robot_module, path, message)
  end

  defp check_state_allowed(command, state) do
    if state.state in command.allowed_states do
      :ok
    else
      {:error,
       StateError.exception(current_state: state.state, allowed_states: command.allowed_states)}
    end
  end

  defp maybe_preempt(_command, %{current_task: nil} = state) do
    {:ok, state}
  end

  defp maybe_preempt(command, state) do
    if :executing in command.allowed_states do
      state = terminate_current_task(state)
      {:ok, state}
    else
      {:error,
       StateError.exception(current_state: :executing, allowed_states: command.allowed_states)}
    end
  end

  defp terminate_current_task(%{current_task: nil} = state), do: state

  defp terminate_current_task(state) do
    # Broadcast cancelled event before killing the task
    # so the waiting caller can return
    path = [:command, state.current_command_name, state.current_execution_id]
    publish_command_event(state.robot_module, path, :cancelled, %{})

    Task.Supervisor.terminate_child(
      task_supervisor_name(state.robot_module),
      state.current_task.pid
    )

    Process.demonitor(state.current_task_ref, [:flush])

    %{
      state
      | current_task: nil,
        current_task_ref: nil,
        current_command_name: nil,
        current_execution_id: nil
    }
  end

  defp publish_transition(state, from, to) do
    message = Message.new!(Transition, :state_machine, from: from, to: to)
    PubSub.publish(state.robot_module, [:state_machine], message)
  end

  defp set_robot_machine_state(state, new_robot_state) do
    RobotState.set_robot_state(state.robot_state, new_robot_state)
    %{state | state: new_robot_state}
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
end
