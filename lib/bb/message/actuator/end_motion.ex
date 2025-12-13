# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.EndMotion do
  @moduledoc """
  Message published by actuators when motion ends.

  Optional counterpart to `BeginMotion`. Useful for actuators with partial
  feedback (limit switches, stall detection) that can report when motion
  completes but may not have continuous position sensing.

  ## Fields

  - `position` - Position when motion ended (radians or metres)
  - `reason` - Why motion ended (`:completed`, `:cancelled`, `:limit_reached`, `:fault`)
  - `detail` - Optional atom with additional context (e.g. `:end_stop`, `:stall`)
  - `message` - Optional human-readable information for operators
  - `command_id` - Optional correlation ID from the originating command

  ## Examples

      alias BB.Message
      alias BB.Message.Actuator.EndMotion

      # Simple completion
      {:ok, msg} = Message.new(EndMotion, :shoulder,
        position: 1.57,
        reason: :completed
      )

      # Limit reached with detail
      {:ok, msg} = Message.new(EndMotion, :shoulder,
        position: 0.0,
        reason: :limit_reached,
        detail: :end_stop
      )

      # Fault with message
      {:ok, msg} = Message.new(EndMotion, :shoulder,
        position: 0.52,
        reason: :fault,
        detail: :stall,
        message: "Motor stall detected at 30% travel"
      )
  """

  @reasons [:completed, :cancelled, :limit_reached, :fault]

  defstruct [:position, :reason, :detail, :message, :command_id]

  use BB.Message,
    schema: [
      position: [
        type: :float,
        required: true,
        doc: "Position when motion ended (radians or metres)"
      ],
      reason: [
        type: {:in, @reasons},
        required: true,
        doc: "Why motion ended"
      ],
      detail: [
        type: :atom,
        required: false,
        doc: "Additional context about the reason"
      ],
      message: [
        type: :string,
        required: false,
        doc: "Human-readable information for operators"
      ],
      command_id: [
        type: :reference,
        required: false,
        doc: "Correlation ID from originating command"
      ]
    ]

  @type reason :: :completed | :cancelled | :limit_reached | :fault

  @type t :: %__MODULE__{
          position: float(),
          reason: reason(),
          detail: atom() | nil,
          message: String.t() | nil,
          command_id: reference() | nil
        }
end
