# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.Command.Velocity do
  @moduledoc """
  Command to set an actuator's velocity.

  The actuator will move at the specified velocity until stopped, a new
  command is received, or the optional duration expires.

  ## Fields

  - `velocity` - Target velocity (rad/s for revolute, m/s for prismatic)
  - `duration` - Optional duration (milliseconds), nil = until stopped
  - `command_id` - Optional reference for correlating with feedback messages

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.Command.Velocity

      # Continuous velocity
      {:ok, msg} = Message.new(Velocity, :wheel,
        velocity: 1.0
      )

      # Velocity for fixed duration
      {:ok, msg} = Message.new(Velocity, :wheel,
        velocity: 1.0,
        duration: 500
      )
  """

  defstruct [:velocity, :duration, :command_id]

  use BB.Message,
    schema: [
      velocity: [
        type: :float,
        required: true,
        doc: "Target velocity (rad/s or m/s)"
      ],
      duration: [
        type: :pos_integer,
        required: false,
        doc: "Duration (milliseconds), nil = until stopped"
      ],
      command_id: [
        type: :reference,
        required: false,
        doc: "Correlation ID for feedback"
      ]
    ]

  @type t :: %__MODULE__{
          velocity: float(),
          duration: pos_integer() | nil,
          command_id: reference() | nil
        }
end
