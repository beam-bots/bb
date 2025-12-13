# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.Command.Position do
  @moduledoc """
  Command to move an actuator to a target position.

  ## Fields

  - `position` - Target position (radians for revolute, metres for prismatic)
  - `velocity` - Optional velocity hint (rad/s or m/s)
  - `duration` - Optional duration hint (milliseconds)
  - `command_id` - Optional reference for correlating with feedback messages

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.Command.Position

      # Simple position command
      {:ok, msg} = Message.new(Position, :shoulder,
        position: 1.57
      )

      # With velocity hint
      {:ok, msg} = Message.new(Position, :shoulder,
        position: 1.57,
        velocity: 0.5
      )

      # With correlation ID for tracking
      {:ok, msg} = Message.new(Position, :shoulder,
        position: 1.57,
        command_id: make_ref()
      )
  """

  defstruct [:position, :velocity, :duration, :command_id]

  use BB.Message,
    schema: [
      position: [
        type: :float,
        required: true,
        doc: "Target position (radians or metres)"
      ],
      velocity: [
        type: :float,
        required: false,
        doc: "Velocity hint (rad/s or m/s)"
      ],
      duration: [
        type: :pos_integer,
        required: false,
        doc: "Duration hint (milliseconds)"
      ],
      command_id: [
        type: :reference,
        required: false,
        doc: "Correlation ID for feedback"
      ]
    ]

  @type t :: %__MODULE__{
          position: float(),
          velocity: float() | nil,
          duration: pos_integer() | nil,
          command_id: reference() | nil
        }
end
