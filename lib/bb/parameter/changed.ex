# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Changed do
  @moduledoc """
  Payload type for parameter change events.

  Published via PubSub when a parameter value changes.
  """

  defstruct [:path, :old_value, :new_value, :source]

  use BB.Message,
    schema: [
      path: [
        type: {:list, :atom},
        required: true,
        doc: "The parameter path that changed"
      ],
      old_value: [
        type: :any,
        required: false,
        doc: "The previous value (nil if newly created)"
      ],
      new_value: [
        type: :any,
        required: true,
        doc: "The new value"
      ],
      source: [
        type: {:in, [:local, :remote, :init, :persisted]},
        required: false,
        default: :local,
        doc: "The source of the change"
      ]
    ]

  @type t :: %__MODULE__{
          path: [atom()],
          old_value: term(),
          new_value: term(),
          source: :local | :remote | :init | :persisted
        }
end
