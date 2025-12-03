# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.StateMachine.Transition do
  @moduledoc """
  Payload type for state machine transition events.
  """

  @behaviour Kinetix.Message

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

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end
end
