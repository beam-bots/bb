# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.CommandPending do
  @moduledoc """
  An actuator has not answered a command within the caller's timeout.

  `delivery: :pubsub` waits for the actuator to accept or refuse a command, and
  this is what the caller gets back when it stops waiting first. It is not a
  rollback: the command is already in the actuator's mailbox, and it will
  almost certainly be applied — late — whatever the caller does next. Sending
  the next target on top of it queues behind it rather than replacing it.

  Its severity is `:warning` because nothing has failed yet; the outcome is
  still to come, and is not reported anywhere the caller can wait on.
  """
  use BB.Error,
    class: :state,
    fields: [:robot, :actuator, :command, :timeout_ms]

  @type t :: %__MODULE__{
          robot: module() | nil,
          actuator: atom(),
          command: module(),
          timeout_ms: non_neg_integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :warning
  end

  def message(%{actuator: actuator, command: command, timeout_ms: timeout}) do
    "Actuator #{inspect(actuator)} has not answered #{inspect(command)} within #{timeout}ms. " <>
      "The command is still queued and will most likely be applied"
  end
end
