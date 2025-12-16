# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Motion.Tracker do
  @moduledoc """
  Behaviour for continuous position tracking with IK.

  Trackers maintain an ongoing IK solution loop, continuously solving for
  updated targets and sending actuator commands. This is useful for:

  - Following a moving target (visual tracking)
  - Smooth trajectory interpolation
  - Real-time position control from external sources

  ## Implementing a Tracker

  Trackers are typically GenServers that:
  1. Run a periodic solve loop at a configurable rate
  2. Accept target updates via `update_target/2`
  3. Send actuator commands on each successful solve
  4. Report status including tracking error and solve statistics

  ## Callbacks

  - `start_tracking/5` - Begin tracking a target link
  - `update_target/2` - Update the current target position
  - `status/1` - Get current tracking status
  - `stop_tracking/2` - Stop tracking and optionally return final positions

  ## Example Implementation

  See `BB.IK.FABRIK.Tracker` for a reference implementation.

  ## Usage Pattern

      # Start tracking
      {:ok, tracker} = BB.IK.FABRIK.Tracker.start_link(
        robot: MyRobot,
        target_link: :gripper,
        initial_target: {0.3, 0.2, 0.1},
        update_rate: 30
      )

      # Update target from vision callback
      BB.IK.FABRIK.Tracker.update_target(tracker, new_target)

      # Check status
      %{residual: 0.001, tracking: true} = BB.IK.FABRIK.Tracker.status(tracker)

      # Stop and get final positions
      {:ok, positions} = BB.IK.FABRIK.Tracker.stop(tracker)
  """

  @type tracker_state :: term()
  @type target :: BB.IK.Solver.target()
  @type positions :: BB.IK.Solver.positions()

  @type status :: %{
          tracking: boolean(),
          target: target() | nil,
          residual: float() | nil,
          iterations: non_neg_integer(),
          update_rate: pos_integer(),
          last_update: DateTime.t() | nil
        }

  @doc """
  Start tracking a target link.

  ## Options

  Required:
  - `:robot` - Robot module or struct
  - `:target_link` - Name of the link to track
  - `:initial_target` - Starting target position

  Optional:
  - `:update_rate` - Solve frequency in Hz (default: 20)
  - `:delivery` - Actuator command delivery mode (default: `:direct`)
  - `:max_iterations` - Maximum solver iterations per update
  - `:tolerance` - Convergence tolerance

  ## Returns

  - `{:ok, state}` - Tracking started
  - `{:error, reason}` - Failed to start
  """
  @callback start_tracking(
              robot :: module() | BB.Robot.t(),
              robot_state :: BB.Robot.State.t(),
              target_link :: atom(),
              initial_target :: target(),
              opts :: keyword()
            ) :: {:ok, tracker_state()} | {:error, term()}

  @doc """
  Update the current target position.

  The tracker will solve for the new target on its next update cycle.

  ## Returns

  - `{:ok, state}` - Target updated
  - `{:error, reason}` - Update failed (e.g., tracker stopped)
  """
  @callback update_target(state :: tracker_state(), target :: target()) ::
              {:ok, tracker_state()} | {:error, term()}

  @doc """
  Get current tracking status.

  ## Returns

  Status map containing:
  - `tracking` - Whether actively tracking
  - `target` - Current target position
  - `residual` - Distance from end-effector to target
  - `iterations` - Solver iterations on last update
  - `update_rate` - Current update frequency
  - `last_update` - Timestamp of last successful solve
  """
  @callback status(state :: tracker_state()) :: status()

  @doc """
  Stop tracking.

  ## Options

  - `:hold` - Whether to send hold commands to actuators (default: false)

  ## Returns

  - `{:ok, positions}` - Final joint positions
  - `{:error, reason}` - Stop failed
  """
  @callback stop_tracking(state :: tracker_state(), opts :: keyword()) ::
              {:ok, positions()} | {:error, term()}
end
