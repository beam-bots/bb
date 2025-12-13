# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.Command.Trajectory do
  @moduledoc """
  Command an actuator to follow a trajectory defined by waypoints.

  A trajectory specifies exact position, velocity, and acceleration at
  each point in time, enabling smooth coordinated motion.

  ## Fields

  - `waypoints` - List of waypoint maps defining the trajectory
  - `repeat` - Number of times to repeat: positive integer or `:forever` (default 1)
  - `command_id` - Optional reference for correlating with feedback messages

  ## Waypoint Structure

  Each waypoint is a map with:

  - `position` - Position at this waypoint (radians or metres)
  - `velocity` - Velocity at this waypoint (rad/s or m/s)
  - `acceleration` - Acceleration at this waypoint (rad/s² or m/s²)
  - `time_from_start` - Time from trajectory start (milliseconds)

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.Command.Trajectory

      waypoints = [
        %{position: 0.0, velocity: 0.0, acceleration: 0.5, time_from_start: 0},
        %{position: 0.1, velocity: 0.3, acceleration: 0.2, time_from_start: 100},
        %{position: 0.3, velocity: 0.4, acceleration: 0.0, time_from_start: 200},
        %{position: 0.5, velocity: 0.3, acceleration: -0.2, time_from_start: 300},
        %{position: 0.6, velocity: 0.0, acceleration: -0.5, time_from_start: 400}
      ]

      {:ok, msg} = Message.new(Trajectory, :shoulder,
        waypoints: waypoints
      )

      # Repeat 5 times
      {:ok, msg} = Message.new(Trajectory, :shoulder,
        waypoints: waypoints,
        repeat: 5
      )

      # Repeat forever (until stopped)
      {:ok, msg} = Message.new(Trajectory, :shoulder,
        waypoints: waypoints,
        repeat: :forever
      )
  """

  defstruct [:waypoints, :repeat, :command_id]

  @waypoint_schema [
    position: [type: :float, required: true, doc: "Position (radians or metres)"],
    velocity: [type: :float, required: true, doc: "Velocity (rad/s or m/s)"],
    acceleration: [type: :float, required: true, doc: "Acceleration (rad/s² or m/s²)"],
    time_from_start: [
      type: :non_neg_integer,
      required: true,
      doc: "Time from start (milliseconds)"
    ]
  ]

  use BB.Message,
    schema: [
      waypoints: [
        type: {:list, {:keyword_list, @waypoint_schema}},
        required: true,
        doc: "List of trajectory waypoints"
      ],
      repeat: [
        type: {:or, [:pos_integer, {:in, [:forever]}]},
        required: false,
        default: 1,
        doc: "Number of times to repeat (positive integer or :forever)"
      ],
      command_id: [
        type: :reference,
        required: false,
        doc: "Correlation ID for feedback"
      ]
    ]

  @type waypoint :: %{
          position: float(),
          velocity: float(),
          acceleration: float(),
          time_from_start: non_neg_integer()
        }

  @type t :: %__MODULE__{
          waypoints: [waypoint()],
          repeat: pos_integer() | :forever,
          command_id: reference() | nil
        }
end
