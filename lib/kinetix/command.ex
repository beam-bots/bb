# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Command do
  @moduledoc """
  Behaviour for implementing robot commands.

  Commands follow a Goal → Feedback → Result pattern similar to ROS2 Actions.
  Each command handler implements callbacks for the various lifecycle stages.

  ## Lifecycle

  1. `init/1` - Called when the command server starts, initialise handler state
  2. `handle_goal/3` - Called when a new goal arrives, accept or reject
  3. `handle_execute/2` - Called after goal is accepted, begin execution
  4. `handle_cancel/2` - Called when cancellation is requested
  5. `handle_info/2` - Handle async messages during execution

  ## States

  Commands progress through these states:
  - `:pending` - Goal received, not yet processed
  - `:accepted` - Handler accepted the goal
  - `:executing` - Actively executing
  - `:canceling` - Cancel requested, winding down
  - `:succeeded` - Completed successfully
  - `:aborted` - Failed during execution
  - `:canceled` - Successfully cancelled
  - `:rejected` - Handler rejected the goal

  ## Example

      defmodule NavigateToPose do
        @behaviour Kinetix.Command

        @impl true
        def init(_opts), do: {:ok, %{}}

        @impl true
        def handle_goal(%{target_pose: pose}, robot_state, state) do
          # Validate the goal is achievable
          if reachable?(pose, robot_state) do
            {:accept, %{state | target: pose}}
          else
            {:reject, :unreachable, state}
          end
        end

        @impl true
        def handle_execute(robot_state, state) do
          # Start navigation
          {:executing, state}
        end

        @impl true
        def handle_cancel(robot_state, state) do
          # Gracefully stop
          {:canceled, :stopped, state}
        end

        @impl true
        def handle_info(:arrived, robot_state, state) do
          {:succeeded, :arrived, state}
        end
      end
  """

  alias Kinetix.Robot.State, as: RobotState

  @type goal :: map()
  @type result :: term()
  @type reason :: term()
  @type handler_state :: term()

  @doc """
  Initialise the command handler.

  Called when the robot starts. Return `{:ok, state}` with initial handler state.
  """
  @callback init(opts :: keyword()) :: {:ok, handler_state()}

  @doc """
  Handle an incoming goal.

  Called when a new goal is submitted. The handler should validate the goal
  and decide whether to accept or reject it.

  Returns:
  - `{:accept, new_state}` - Accept the goal, will call `handle_execute/2` next
  - `{:reject, reason, new_state}` - Reject the goal with a reason
  """
  @callback handle_goal(goal(), RobotState.t(), handler_state()) ::
              {:accept, handler_state()}
              | {:reject, reason(), handler_state()}

  @doc """
  Begin executing the accepted goal.

  Called after the goal is accepted. The handler should start any async work
  needed to achieve the goal.

  Returns:
  - `{:executing, new_state}` - Execution in progress, wait for handle_info
  - `{:succeeded, result, new_state}` - Immediately completed successfully
  - `{:aborted, reason, new_state}` - Failed to start execution
  """
  @callback handle_execute(RobotState.t(), handler_state()) ::
              {:executing, handler_state()}
              | {:succeeded, result(), handler_state()}
              | {:aborted, reason(), handler_state()}

  @doc """
  Handle a cancellation request.

  Called when the caller requests cancellation. The handler should begin
  graceful shutdown of the command.

  Returns:
  - `{:canceling, new_state}` - Starting cancellation, wait for handle_info
  - `{:canceled, result, new_state}` - Immediately cancelled
  - `{:aborted, reason, new_state}` - Cannot cancel cleanly, aborted instead
  """
  @callback handle_cancel(RobotState.t(), handler_state()) ::
              {:canceling, handler_state()}
              | {:canceled, result(), handler_state()}
              | {:aborted, reason(), handler_state()}

  @doc """
  Handle async messages during execution.

  Called for any messages sent to the command handler during execution.
  Use this to process sensor data, timer events, or completion notifications.

  Returns:
  - `{:executing, new_state}` - Still executing
  - `{:succeeded, result, new_state}` - Completed successfully
  - `{:aborted, reason, new_state}` - Failed during execution
  - `{:canceled, result, new_state}` - Cancellation completed (only valid when canceling)
  """
  @callback handle_info(msg :: term(), RobotState.t(), handler_state()) ::
              {:executing, handler_state()}
              | {:succeeded, result(), handler_state()}
              | {:aborted, reason(), handler_state()}
              | {:canceled, result(), handler_state()}

  @doc """
  Optional callback for cleanup when command completes.

  Called after the command finishes (succeeded, aborted, or canceled).
  Default implementation does nothing.
  """
  @callback terminate(reason :: term(), handler_state()) :: :ok

  @optional_callbacks terminate: 2
end
