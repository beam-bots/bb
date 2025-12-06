# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.StateMachine.Transition do
  @moduledoc """
  Payload type for state machine transition events.
  """

  @behaviour BB.Message

  defstruct [:from, :to]

  @type t :: %__MODULE__{
          from: atom,
          to: atom
        }

  @schema Spark.Options.new!(
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
          )

  @impl BB.Message
  def schema, do: @schema

  defimpl BB.Message.Payload do
    def schema(_), do: @for.schema()
  end
end
