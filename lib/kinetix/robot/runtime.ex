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
  """

  use GenServer

  alias Kinetix.Command.Execution
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
    :handlers,
    :current_execution
  ]

  @type robot_state :: :disarmed | :idle | :executing
  @type t :: %__MODULE__{
          robot_module: module(),
          robot: Kinetix.Robot.t(),
          robot_state: RobotState.t(),
          state: robot_state(),
          commands: %{atom() => Kinetix.Dsl.Command.t()},
          handlers: %{atom() => {module(), term()}},
          current_execution: Execution.t() | nil
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

  Returns `{:ok, ref}` if the command was accepted, where `ref` can be used
  to track the execution. The caller will receive messages as the command
  progresses.
  """
  @spec execute(module(), atom(), map()) ::
          {:ok, reference()} | {:error, StateError.t() | term()}
  def execute(robot_module, command_name, goal) do
    GenServer.call(via(robot_module), {:execute, command_name, goal})
  end

  @doc """
  Cancel the currently executing command.
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

    handlers = initialise_handlers(commands)

    state = %__MODULE__{
      robot_module: robot_module,
      robot: robot,
      robot_state: robot_state,
      state: :disarmed,
      commands: commands,
      handlers: handlers,
      current_execution: nil
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

  def handle_call({:execute, command_name, goal}, from, state) do
    case Map.fetch(state.commands, command_name) do
      {:ok, command} ->
        handle_execute_command(command, goal, from, state)

      :error ->
        {:reply, {:error, {:unknown_command, command_name}}, state}
    end
  end

  def handle_call(:cancel, _from, %{current_execution: nil} = state) do
    {:reply, {:error, :no_execution}, state}
  end

  def handle_call(:cancel, _from, state) do
    state = handle_cancel_execution(state)
    {:reply, :ok, state}
  end

  @impl GenServer
  def handle_info(msg, %{current_execution: %Execution{} = exec} = state) do
    {handler_mod, handler_state} = Map.fetch!(state.handlers, exec.command_name)

    case handler_mod.handle_info(msg, state.robot_state, handler_state) do
      {:executing, new_handler_state} ->
        state = update_handler_state(state, exec.command_name, new_handler_state)
        {:noreply, state}

      {:succeeded, result, new_handler_state} ->
        state = complete_execution(state, exec, :succeeded, result, new_handler_state)
        {:noreply, state}

      {:aborted, reason, new_handler_state} ->
        state = complete_execution(state, exec, :aborted, reason, new_handler_state)
        {:noreply, state}

      {:canceled, result, new_handler_state} ->
        state = complete_execution(state, exec, :canceled, result, new_handler_state)
        {:noreply, state}
    end
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

  defp handle_execute_command(command, goal, from, state) do
    with :ok <- check_state_allowed(command, state),
         {:ok, state} <- maybe_preempt(command, state) do
      exec = Execution.new(command.name, goal, from)
      {handler_mod, handler_state} = Map.fetch!(state.handlers, command.name)

      case handler_mod.handle_goal(goal, state.robot_state, handler_state) do
        {:accept, new_handler_state} ->
          state = update_handler_state(state, command.name, new_handler_state)
          state = start_execution(state, exec, handler_mod, new_handler_state)
          {:reply, {:ok, exec.id}, state}

        {:reject, reason, new_handler_state} ->
          state = update_handler_state(state, command.name, new_handler_state)
          {:reply, {:error, {:rejected, reason}}, state}
      end
    else
      {:error, _} = err ->
        {:reply, err, state}
    end
  end

  defp check_state_allowed(command, state) do
    if state.state in command.allowed_states do
      :ok
    else
      {:error,
       StateError.exception(current_state: state.state, allowed_states: command.allowed_states)}
    end
  end

  defp maybe_preempt(_command, %{current_execution: nil} = state) do
    {:ok, state}
  end

  defp maybe_preempt(command, state) do
    if :executing in command.allowed_states do
      state = handle_cancel_execution(state)
      {:ok, state}
    else
      {:error,
       StateError.exception(current_state: :executing, allowed_states: command.allowed_states)}
    end
  end

  defp start_execution(state, exec, handler_mod, handler_state) do
    case handler_mod.handle_execute(state.robot_state, handler_state) do
      {:executing, new_handler_state} ->
        exec = %{exec | status: :executing, handler_state: new_handler_state}
        state = update_handler_state(state, exec.command_name, new_handler_state)

        old_state = state.state
        state = set_robot_machine_state(state, :executing)
        state = %{state | current_execution: exec}

        if old_state != :executing do
          publish_transition(state, old_state, :executing)
        end

        state

      {:succeeded, result, new_handler_state} ->
        reply_to_caller(exec.caller, {:ok, result})
        update_handler_state(state, exec.command_name, new_handler_state)

      {:aborted, reason, new_handler_state} ->
        reply_to_caller(exec.caller, {:error, {:aborted, reason}})
        update_handler_state(state, exec.command_name, new_handler_state)
    end
  end

  defp handle_cancel_execution(%{current_execution: nil} = state), do: state

  defp handle_cancel_execution(state) do
    exec = state.current_execution
    {handler_mod, handler_state} = Map.fetch!(state.handlers, exec.command_name)

    case handler_mod.handle_cancel(state.robot_state, handler_state) do
      {:canceling, new_handler_state} ->
        exec = %{exec | status: :canceling}
        state = update_handler_state(state, exec.command_name, new_handler_state)
        %{state | current_execution: exec}

      {:canceled, result, new_handler_state} ->
        complete_execution(state, exec, :canceled, result, new_handler_state)

      {:aborted, reason, new_handler_state} ->
        complete_execution(state, exec, :aborted, reason, new_handler_state)
    end
  end

  defp complete_execution(state, exec, status, result, new_handler_state) do
    reply =
      case status do
        :succeeded -> {:ok, result}
        :canceled -> {:ok, {:canceled, result}}
        :aborted -> {:error, {:aborted, result}}
      end

    reply_to_caller(exec.caller, reply)

    state = update_handler_state(state, exec.command_name, new_handler_state)

    old_state = state.state
    state = set_robot_machine_state(state, :idle)
    state = %{state | current_execution: nil}

    if old_state != :idle do
      publish_transition(state, old_state, :idle)
    end

    state
  end

  defp reply_to_caller(from, reply) do
    GenServer.reply(from, reply)
  end

  defp update_handler_state(state, command_name, new_handler_state) do
    {handler_mod, _old_state} = Map.fetch!(state.handlers, command_name)
    handlers = Map.put(state.handlers, command_name, {handler_mod, new_handler_state})
    %{state | handlers: handlers}
  end

  defp initialise_handlers(commands) do
    Map.new(commands, fn {name, command} ->
      {:ok, handler_state} = command.handler.init([])
      {name, {command.handler, handler_state}}
    end)
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
