# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.HardwareError do
  @moduledoc """
  Payload type for hardware error events.

  Published to `[:safety, :error]` when a component reports a hardware error.
  Subscribe to receive notifications of hardware failures.

  ## Example

      BB.subscribe(MyRobot, [:safety, :error])

      # Receive:
      # {:bb, [:safety, :error], %BB.Message{payload: %BB.Safety.HardwareError{...}}}
  """

  defstruct [:path, :error]

  use BB.Message,
    schema: [
      path: [
        type: {:list, :atom},
        required: true,
        doc: "Path to the component that reported the error"
      ],
      error: [
        type: :any,
        required: true,
        doc: "The error details (component-specific)"
      ]
    ]

  @type t :: %__MODULE__{
          path: [atom],
          error: term
        }
end
