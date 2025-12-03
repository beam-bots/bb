# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.StateMachine do
  @moduledoc """
  Tracks the robot's operational state.

  The state machine controls which commands can run based on the current state.
  Commands declare their `allowed_states` and are rejected if the current state
  is not in that list.

  ## Default States

  - `:disarmed` - Robot is not armed; most commands are blocked
  - `:idle` - Robot is armed and ready to execute commands
  - `:executing` - A command is currently running

  ## State Transitions

  ```
  :disarmed ──arm──→ :idle
  :idle ──execute──→ :executing
  :executing ──complete──→ :idle
  :executing ──disarm──→ :disarmed
  :idle ──disarm──→ :disarmed
  ```

  ## Usage

      # Get current state
      Kinetix.StateMachine.state(MyRobot)
      # => :disarmed

      # Transition to a new state
      {:ok, :idle} = Kinetix.StateMachine.transition(MyRobot, :idle)

      # Check if a command can run
      :ok = Kinetix.StateMachine.check_allowed(MyRobot, [:idle, :executing])
      # => :ok or {:error, %StateError{}}
  """

  use GenServer
  alias Kinetix.{Message, PubSub}
  alias Kinetix.StateMachine.Transition

  @type state :: atom
  @type robot :: module

  defmodule StateError do
    @moduledoc """
    Error raised when a command cannot run in the current state.
    """
    defexception [:current_state, :allowed_states, :message]

    @type t :: %__MODULE__{
            current_state: atom,
            allowed_states: [atom],
            message: String.t()
          }

    @impl true
    def exception(opts) do
      current = Keyword.fetch!(opts, :current_state)
      allowed = Keyword.fetch!(opts, :allowed_states)

      msg = """
      Cannot execute command in current state.

      Current state: #{inspect(current)}
      Allowed states: #{inspect(allowed)}
      """

      %__MODULE__{
        current_state: current,
        allowed_states: allowed,
        message: msg
      }
    end
  end

  @doc """
  Starts the state machine for a robot.
  """
  @spec start_link({robot, Keyword.t()}) :: GenServer.on_start()
  def start_link({robot, opts}) do
    initial_state = Keyword.get(opts, :initial_state, :disarmed)
    GenServer.start_link(__MODULE__, {robot, initial_state}, name: via(robot))
  end

  @doc """
  Returns the current state of the robot.
  """
  @spec state(robot) :: state
  def state(robot) do
    GenServer.call(via(robot), :state)
  end

  @doc """
  Transitions to a new state.

  Returns `{:ok, new_state}` on success.
  """
  @spec transition(robot, state) :: {:ok, state}
  def transition(robot, new_state) do
    GenServer.call(via(robot), {:transition, new_state})
  end

  @doc """
  Checks if the current state allows the given states.

  Returns `:ok` if the current state is in `allowed_states`, otherwise
  returns `{:error, %StateError{}}`.
  """
  @spec check_allowed(robot, [state]) :: :ok | {:error, StateError.t()}
  def check_allowed(robot, allowed_states) do
    current = state(robot)

    if current in allowed_states do
      :ok
    else
      {:error, StateError.exception(current_state: current, allowed_states: allowed_states)}
    end
  end

  @doc """
  Returns the via tuple for the state machine process.
  """
  @spec via(robot) :: GenServer.name()
  def via(robot) do
    Kinetix.Process.via(robot, :__state_machine__)
  end

  # GenServer callbacks

  @impl true
  def init({robot, initial_state}) do
    {:ok, %{robot: robot, state: initial_state}}
  end

  @impl true
  def handle_call(:state, _from, state) do
    {:reply, state.state, state}
  end

  @impl true
  def handle_call({:transition, new_state}, _from, state) do
    old_state = state.state

    if old_state != new_state do
      message = Message.new!(Transition, :state_machine, from: old_state, to: new_state)
      PubSub.publish(state.robot, [:state_machine], message)
    end

    {:reply, {:ok, new_state}, %{state | state: new_state}}
  end
end
