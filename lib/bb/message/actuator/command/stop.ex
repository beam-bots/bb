# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.Command.Stop do
  @moduledoc """
  Command to stop an actuator's motion.

  After stopping, the actuator becomes passive and will not actively resist
  external forces. Use `Hold` if you need the actuator to maintain position.

  ## Fields

  - `mode` - Stop mode: `:immediate` (default) or `:decelerate`
  - `command_id` - Optional reference for correlating with feedback messages

  ## Modes

  - `:immediate` - Stop as quickly as possible (may be abrupt)
  - `:decelerate` - Slow down smoothly before stopping

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.Command.Stop

      # Immediate stop
      {:ok, msg} = Message.new(Stop, :shoulder, [])

      # Decelerate to stop
      {:ok, msg} = Message.new(Stop, :shoulder,
        mode: :decelerate
      )
  """

  @modes [:immediate, :decelerate]

  defstruct mode: :immediate, command_id: nil

  use BB.Message,
    schema: [
      mode: [
        type: {:in, @modes},
        required: false,
        default: :immediate,
        doc: "Stop mode: :immediate or :decelerate"
      ],
      command_id: [
        type: {:or, [nil, :reference]},
        required: false,
        doc: "Correlation ID for feedback"
      ]
    ]

  @type mode :: :immediate | :decelerate

  @type t :: %__MODULE__{
          mode: mode(),
          command_id: reference() | nil
        }
end
