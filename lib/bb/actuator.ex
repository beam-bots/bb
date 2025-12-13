# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator do
  @moduledoc """
  Interface for sending commands to actuators.

  Supports both pubsub delivery (for orchestration, logging, replay) and
  direct GenServer delivery (for time-critical control paths).

  ## Delivery Methods

  - **Pubsub** (`set_position/4`, etc.) - Commands published to `[:actuator | path]`.
    Enables logging, replay, and multi-subscriber patterns. Actuators receive
    commands via `handle_info/2`.

  - **Direct** (`set_position!/4`, etc.) - Commands sent directly via `BB.Process.cast`.
    Lower latency for time-critical control. Actuators receive via `handle_cast/2`.

  - **Synchronous** (`set_position_sync/5`, etc.) - Commands sent via `BB.Process.call`.
    Returns acknowledgement or error. Actuators respond via `handle_call/3`.

  ## Examples

      # Pubsub delivery (for kinematics/orchestration)
      BB.Actuator.set_position(MyRobot, [:base_link, :shoulder, :servo], 1.57)

      # Direct delivery (for time-critical control)
      BB.Actuator.set_position!(MyRobot, :shoulder_servo, 1.57)

      # Synchronous with acknowledgement
      {:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :shoulder_servo, 1.57)
  """

  alias BB.Message
  alias BB.Message.Actuator.Command

  # ----------------------------------------------------------------------------
  # Position Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a position command via pubsub.

  The command is published to `[:actuator | path]` where subscribers can
  receive it via `handle_info({:bb, path, message}, state)`.

  ## Options

  - `:velocity` - Velocity hint (rad/s or m/s)
  - `:duration` - Duration hint (milliseconds)
  - `:command_id` - Correlation ID for feedback tracking

  ## Examples

      BB.Actuator.set_position(MyRobot, [:base_link, :shoulder, :servo], 1.57)
      BB.Actuator.set_position(MyRobot, [:shoulder, :servo], 1.57, velocity: 0.5)
  """
  @spec set_position(module(), [atom()], number(), keyword()) :: :ok
  def set_position(robot, path, position, opts \\ []) do
    message = build_position_message(path, position, opts)
    BB.publish(robot, [:actuator | path], message)
  end

  @doc """
  Send a position command directly to an actuator (bypasses pubsub).

  Uses `BB.Process.cast` for fire-and-forget delivery. The actuator receives
  the command via `handle_cast({:command, message}, state)`.

  ## Options

  Same as `set_position/4`.
  """
  @spec set_position!(module(), atom(), number(), keyword()) :: :ok
  def set_position!(robot, actuator_name, position, opts \\ []) do
    message = build_position_message(actuator_name, position, opts)
    BB.cast(robot, actuator_name, {:command, message})
  end

  @doc """
  Send a position command and wait for acknowledgement.

  Uses `BB.Process.call` for synchronous delivery. Returns the actuator's
  response or raises on timeout.

  ## Options

  Same as `set_position/4`, plus:
  - Fifth argument is timeout in milliseconds (default 5000)

  ## Returns

  - `{:ok, :accepted}` - Command accepted
  - `{:ok, :accepted, map()}` - Command accepted with additional info
  - `{:error, reason}` - Command rejected
  """
  @spec set_position_sync(module(), atom(), number(), keyword(), timeout()) ::
          {:ok, :accepted | {:accepted, map()}} | {:error, term()}
  def set_position_sync(robot, actuator_name, position, opts \\ [], timeout \\ 5000) do
    message = build_position_message(actuator_name, position, opts)
    BB.call(robot, actuator_name, {:command, message}, timeout)
  end

  defp build_position_message(frame_id, position, opts) do
    frame_id = if is_list(frame_id), do: List.last(frame_id), else: frame_id

    Message.new!(Command.Position, frame_id,
      position: position * 1.0,
      velocity: opts[:velocity],
      duration: opts[:duration],
      command_id: opts[:command_id]
    )
  end

  # ----------------------------------------------------------------------------
  # Velocity Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a velocity command via pubsub.

  ## Options

  - `:duration` - Duration (milliseconds), nil = until stopped
  - `:command_id` - Correlation ID for feedback tracking
  """
  @spec set_velocity(module(), [atom()], number(), keyword()) :: :ok
  def set_velocity(robot, path, velocity, opts \\ []) do
    message = build_velocity_message(path, velocity, opts)
    BB.publish(robot, [:actuator | path], message)
  end

  @doc """
  Send a velocity command directly to an actuator (bypasses pubsub).
  """
  @spec set_velocity!(module(), atom(), number(), keyword()) :: :ok
  def set_velocity!(robot, actuator_name, velocity, opts \\ []) do
    message = build_velocity_message(actuator_name, velocity, opts)
    BB.cast(robot, actuator_name, {:command, message})
  end

  @doc """
  Send a velocity command and wait for acknowledgement.
  """
  @spec set_velocity_sync(module(), atom(), number(), keyword(), timeout()) ::
          {:ok, :accepted | {:accepted, map()}} | {:error, term()}
  def set_velocity_sync(robot, actuator_name, velocity, opts \\ [], timeout \\ 5000) do
    message = build_velocity_message(actuator_name, velocity, opts)
    BB.call(robot, actuator_name, {:command, message}, timeout)
  end

  defp build_velocity_message(frame_id, velocity, opts) do
    frame_id = if is_list(frame_id), do: List.last(frame_id), else: frame_id

    Message.new!(Command.Velocity, frame_id,
      velocity: velocity * 1.0,
      duration: opts[:duration],
      command_id: opts[:command_id]
    )
  end

  # ----------------------------------------------------------------------------
  # Effort Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send an effort (torque/force) command via pubsub.

  ## Options

  - `:duration` - Duration (milliseconds), nil = until stopped
  - `:command_id` - Correlation ID for feedback tracking
  """
  @spec set_effort(module(), [atom()], number(), keyword()) :: :ok
  def set_effort(robot, path, effort, opts \\ []) do
    message = build_effort_message(path, effort, opts)
    BB.publish(robot, [:actuator | path], message)
  end

  @doc """
  Send an effort command directly to an actuator (bypasses pubsub).
  """
  @spec set_effort!(module(), atom(), number(), keyword()) :: :ok
  def set_effort!(robot, actuator_name, effort, opts \\ []) do
    message = build_effort_message(actuator_name, effort, opts)
    BB.cast(robot, actuator_name, {:command, message})
  end

  @doc """
  Send an effort command and wait for acknowledgement.
  """
  @spec set_effort_sync(module(), atom(), number(), keyword(), timeout()) ::
          {:ok, :accepted | {:accepted, map()}} | {:error, term()}
  def set_effort_sync(robot, actuator_name, effort, opts \\ [], timeout \\ 5000) do
    message = build_effort_message(actuator_name, effort, opts)
    BB.call(robot, actuator_name, {:command, message}, timeout)
  end

  defp build_effort_message(frame_id, effort, opts) do
    frame_id = if is_list(frame_id), do: List.last(frame_id), else: frame_id

    Message.new!(Command.Effort, frame_id,
      effort: effort * 1.0,
      duration: opts[:duration],
      command_id: opts[:command_id]
    )
  end

  # ----------------------------------------------------------------------------
  # Trajectory Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a trajectory command via pubsub.

  ## Waypoint Structure

  Each waypoint should be a keyword list or map with:
  - `position` - Position (radians or metres)
  - `velocity` - Velocity (rad/s or m/s)
  - `acceleration` - Acceleration (rad/s² or m/s²)
  - `time_from_start` - Time from trajectory start (milliseconds)

  ## Options

  - `:repeat` - Number of repetitions: positive integer or `:forever` (default 1)
  - `:command_id` - Correlation ID for feedback tracking
  """
  @spec follow_trajectory(module(), [atom()], [keyword() | map()], keyword()) :: :ok
  def follow_trajectory(robot, path, waypoints, opts \\ []) do
    message = build_trajectory_message(path, waypoints, opts)
    BB.publish(robot, [:actuator | path], message)
  end

  @doc """
  Send a trajectory command directly to an actuator (bypasses pubsub).
  """
  @spec follow_trajectory!(module(), atom(), [keyword() | map()], keyword()) :: :ok
  def follow_trajectory!(robot, actuator_name, waypoints, opts \\ []) do
    message = build_trajectory_message(actuator_name, waypoints, opts)
    BB.cast(robot, actuator_name, {:command, message})
  end

  @doc """
  Send a trajectory command and wait for acknowledgement.
  """
  @spec follow_trajectory_sync(module(), atom(), [keyword() | map()], keyword(), timeout()) ::
          {:ok, :accepted | {:accepted, map()}} | {:error, term()}
  def follow_trajectory_sync(robot, actuator_name, waypoints, opts \\ [], timeout \\ 5000) do
    message = build_trajectory_message(actuator_name, waypoints, opts)
    BB.call(robot, actuator_name, {:command, message}, timeout)
  end

  defp build_trajectory_message(frame_id, waypoints, opts) do
    frame_id = if is_list(frame_id), do: List.last(frame_id), else: frame_id

    normalised_waypoints =
      Enum.map(waypoints, fn wp ->
        wp = if is_map(wp), do: Keyword.new(wp), else: wp

        [
          position: wp[:position] * 1.0,
          velocity: wp[:velocity] * 1.0,
          acceleration: wp[:acceleration] * 1.0,
          time_from_start: wp[:time_from_start]
        ]
      end)

    Message.new!(Command.Trajectory, frame_id,
      waypoints: normalised_waypoints,
      repeat: opts[:repeat] || 1,
      command_id: opts[:command_id]
    )
  end

  # ----------------------------------------------------------------------------
  # Stop Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a stop command via pubsub.

  ## Options

  - `:mode` - `:immediate` (default) or `:decelerate`
  - `:command_id` - Correlation ID for feedback tracking
  """
  @spec stop(module(), [atom()], keyword()) :: :ok
  def stop(robot, path, opts \\ []) do
    message = build_stop_message(path, opts)
    BB.publish(robot, [:actuator | path], message)
  end

  @doc """
  Send a stop command directly to an actuator (bypasses pubsub).
  """
  @spec stop!(module(), atom(), keyword()) :: :ok
  def stop!(robot, actuator_name, opts \\ []) do
    message = build_stop_message(actuator_name, opts)
    BB.cast(robot, actuator_name, {:command, message})
  end

  @doc """
  Send a stop command and wait for acknowledgement.
  """
  @spec stop_sync(module(), atom(), keyword(), timeout()) ::
          {:ok, :accepted | {:accepted, map()}} | {:error, term()}
  def stop_sync(robot, actuator_name, opts \\ [], timeout \\ 5000) do
    message = build_stop_message(actuator_name, opts)
    BB.call(robot, actuator_name, {:command, message}, timeout)
  end

  defp build_stop_message(frame_id, opts) do
    frame_id = if is_list(frame_id), do: List.last(frame_id), else: frame_id

    Message.new!(Command.Stop, frame_id,
      mode: opts[:mode] || :immediate,
      command_id: opts[:command_id]
    )
  end

  # ----------------------------------------------------------------------------
  # Hold Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a hold command via pubsub.

  Instructs the actuator to actively maintain its current position.

  ## Options

  - `:command_id` - Correlation ID for feedback tracking
  """
  @spec hold(module(), [atom()], keyword()) :: :ok
  def hold(robot, path, opts \\ []) do
    message = build_hold_message(path, opts)
    BB.publish(robot, [:actuator | path], message)
  end

  @doc """
  Send a hold command directly to an actuator (bypasses pubsub).
  """
  @spec hold!(module(), atom(), keyword()) :: :ok
  def hold!(robot, actuator_name, opts \\ []) do
    message = build_hold_message(actuator_name, opts)
    BB.cast(robot, actuator_name, {:command, message})
  end

  @doc """
  Send a hold command and wait for acknowledgement.
  """
  @spec hold_sync(module(), atom(), keyword(), timeout()) ::
          {:ok, :accepted | {:accepted, map()}} | {:error, term()}
  def hold_sync(robot, actuator_name, opts \\ [], timeout \\ 5000) do
    message = build_hold_message(actuator_name, opts)
    BB.call(robot, actuator_name, {:command, message}, timeout)
  end

  defp build_hold_message(frame_id, opts) do
    frame_id = if is_list(frame_id), do: List.last(frame_id), else: frame_id
    Message.new!(Command.Hold, frame_id, command_id: opts[:command_id])
  end
end
