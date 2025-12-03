# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Command do
  @moduledoc """
  Behaviour for implementing robot commands.

  Commands execute in a supervised task and receive a goal (arguments) and
  context (robot state). The handler runs to completion and returns a result.

  ## Example

      defmodule NavigateToPose do
        @behaviour Kinetix.Command

        @impl true
        def handle_command(%{target_pose: pose, tolerance: tol}, context) do
          # Access robot state
          current_pose = get_current_pose(context.robot_state)

          # Do the work (this runs in a task, so blocking is fine)
          case navigate_to(pose, tolerance: tol) do
            :ok -> {:ok, %{final_pose: pose}}
            {:error, reason} -> {:error, reason}
          end
        end
      end

  ## Execution Model

  Commands run in supervised tasks spawned by the Runtime. The caller receives
  a `Task.t()` and can use `Task.await/2` or `Task.yield/2` to get the result.

  Cancellation is handled by killing the task - handlers don't need to implement
  cancellation logic. If graceful shutdown is needed, handlers can trap exits.
  """

  alias Kinetix.Command.Context

  @type goal :: map()
  @type result :: term()

  @doc """
  Execute the command with the given goal.

  Called in a supervised task. The handler should perform the work and return
  the result. Blocking operations are fine since this runs in a separate process.

  The context provides access to:
  - `robot_module` - The robot module
  - `robot` - The static robot struct (topology)
  - `robot_state` - The dynamic robot state (ETS-backed joint positions etc)
  - `execution_id` - Unique identifier for this execution

  Returns:
  - `{:ok, result}` - Command succeeded with result
  - `{:error, reason}` - Command failed with reason
  """
  @callback handle_command(goal(), Context.t()) :: {:ok, result()} | {:error, term()}
end
