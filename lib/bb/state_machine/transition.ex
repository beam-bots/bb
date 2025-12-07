# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.StateMachine.Transition do
  @moduledoc """
  Payload type for state machine transition events.
  """

  defstruct [:from, :to]

  use BB.Message,
    schema: [
      from: [
        type: :atom,
        required: true,
        doc: "The state being transitioned from"
      ],
      to: [
        type: :atom,
        required: true,
        doc: "The state being transitioned to"
      ]
    ]

  @type t :: %__MODULE__{
          from: atom,
          to: atom
        }
end
