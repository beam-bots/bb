# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.UnsupportedCommand do
  @moduledoc """
  Command refused because the actuator doesn't accept that payload.

  An actuator declares what it accepts with `c:BB.Actuator.command_payloads/1`,
  defaulting to the six built-in `BB.Message.Actuator.Command` types. A driver
  that narrows the list — a port that only speaks effort, say — gets everything
  else refused here rather than handed to it to misinterpret.

  Published commands are filtered out by the subscription and never reach the
  actuator at all; this error is what the direct and synchronous transports see,
  since they bypass pubsub.
  """
  use BB.Error,
    class: :state,
    fields: [:robot, :actuator, :command, :supported]

  @type t :: %__MODULE__{
          robot: module() | nil,
          actuator: atom(),
          command: module(),
          supported: [module()]
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{actuator: actuator, command: command, supported: supported}) do
    "Actuator #{inspect(actuator)} does not accept #{inspect(command)}. " <>
      "Accepts: #{inspect(supported)}"
  end
end
