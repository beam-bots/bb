# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Command.Event do
  @moduledoc """
  Payload type for command execution events.

  Published to `[:command, command_name, execution_id]` path during command lifecycle.
  """

  @behaviour Kinetix.Message

  defstruct [:status, :data]

  @type status :: :started | :succeeded | :failed | :cancelled
  @type t :: %__MODULE__{
          status: status(),
          data: map()
        }

  @schema Spark.Options.new!(
            status: [
              type: {:in, [:started, :succeeded, :failed, :cancelled]},
              required: true,
              doc: "The command execution status"
            ],
            data: [
              type: :map,
              default: %{},
              doc: "Additional data associated with the event"
            ]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end
end
