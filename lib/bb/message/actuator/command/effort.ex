# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.Command.Effort do
  @moduledoc """
  Command to apply a specific effort (torque/force) to an actuator.

  The actuator will apply the specified effort until stopped, a new
  command is received, or the optional duration expires.

  ## Fields

  - `effort` - Target effort (Nm for revolute, N for prismatic)
  - `duration` - Optional duration (milliseconds), nil = until stopped
  - `command_id` - Optional reference for correlating with feedback messages

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.Command.Effort

      # Apply torque
      {:ok, msg} = Message.new(Effort, :gripper,
        effort: 0.5
      )

      # Apply torque for fixed duration
      {:ok, msg} = Message.new(Effort, :gripper,
        effort: 0.5,
        duration: 1000
      )
  """

  defstruct [:effort, :duration, :command_id]

  use BB.Message,
    schema: [
      effort: [
        type: :float,
        required: true,
        doc: "Target effort (Nm or N)"
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
          effort: float(),
          duration: pos_integer() | nil,
          command_id: reference() | nil
        }
end
