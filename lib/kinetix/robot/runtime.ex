# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.Runtime do
  @moduledoc """
  Runtime process for a Kinetix robot.

  Manages the robot's runtime state including:
  - The `Kinetix.Robot` struct (static topology)
  - The `Kinetix.Robot.State` ETS table (dynamic joint state)
  - Robot state machine (disarmed/idle/executing)
  - Command execution lifecycle

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

  alias Kinetix.Command.{Context, Event}
  alias Kinetix.Dsl.Info
  alias Kinetix.{Message, PubSub}
  alias Kinetix.Robot.State, as: RobotState
  alias Kinetix.StateMachine.Transition

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
    :current_execution_id
  ]

  @type robot_state :: :disarmed | :idle | :executing
  @type t :: %__MODULE__{
          robot_module: module(),
          robot: Kinetix.Robot.t(),
          robot_state: RobotState.t(),
          state: robot_state(),
          commands: %{atom() => Kinetix.Dsl.Command.t()},
          current_task: Task.t() | nil,
          current_task_ref: reference() | nil,
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
    Kinetix.Process.via(robot_module, __MODULE__)
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
  @spec get_robot(module()) :: Kinetix.Robot.t()
  def get_robot(robot_module) do
    GenServer.call(via(robot_module), :get_robot)
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
      {:kinetix, ^path, %Message{payload: %Event{status: :succeeded, data: %{result: result}}}} ->
        {:ok, result}

      {:kinetix, ^path, %Message{payload: %Event{status: :failed, data: %{reason: reason}}}} ->
        {:error, reason}

      {:kinetix, ^path, %Message{payload: %Event{status: :cancelled}}} ->
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
      |> Info.robot_commands()
      |> Map.new(&{&1.name, &1})

    state = %__MODULE__{
      robot_module: robot_module,
      robot: robot,
      robot_state: robot_state,
      state: :disarmed,
      commands: commands,
      current_task: nil,
      current_task_ref: nil,
      current_command_name: nil,
      current_execution_id: nil
    }

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

  def handle_info(_msg, state) do
    {:noreply, state}
  end

  @impl GenServer
  def terminate(_reason, state) do
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
    Kinetix.Process.via(robot_module, Kinetix.TaskSupervisor)
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
end
