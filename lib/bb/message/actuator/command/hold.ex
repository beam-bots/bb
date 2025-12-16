# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.Command.Hold do
  @moduledoc """
  Command an actuator to actively maintain its current position.

  Unlike `Stop`, the actuator will actively resist external forces to
  stay at its current position. This consumes power but provides rigidity.

  ## Fields

  - `command_id` - Optional reference for correlating with feedback messages

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.Command.Hold

      {:ok, msg} = Message.new(Hold, :shoulder, [])

      # With correlation ID
      {:ok, msg} = Message.new(Hold, :shoulder,
        command_id: make_ref()
      )

  ## Notes

  Not all actuators distinguish between stop and hold. RC servos, for example,
  always hold their position when given a command. This command is most relevant
  for motors with encoders or steppers where passive vs active holding differs.
  """

  defstruct [:command_id]

  use BB.Message,
    schema: [
      command_id: [
        type: {:or, [nil, :reference]},
        required: false,
        doc: "Correlation ID for feedback"
      ]
    ]

  @type t :: %__MODULE__{
          command_id: reference() | nil
        }
end
