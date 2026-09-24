# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.StaleEpoch do
  @moduledoc """
  Command refused because it does not belong to the current arming session.

  `BB.Safety.armed?/1` answers "is the robot armed now?", which is not the
  same question as "has the robot been armed continuously since this command
  was created?". Without the second answer a command created during one
  arming session could still be applied during a later one, after an
  intervening disarm that was supposed to make the robot safe.

  So `BB.Actuator`'s send functions stamp each outgoing command with the arm
  epoch current at the moment it is sent, and `BB.Actuator.Server` refuses it
  if the robot has since moved on. A command carrying no epoch at all is
  refused here too: an unstamped command cannot be shown to belong to the
  session that is running, and hand-built commands delivered straight to an
  actuator are exactly the case the epoch exists to catch.

  This is a `:state` error rather than a `:safety` one: the safety system
  working as intended is not a safety violation.
  """
  use BB.Error,
    class: :state,
    fields: [:robot, :actuator, :command, :epoch, :current_epoch]

  @type t :: %__MODULE__{
          robot: module() | nil,
          actuator: atom(),
          command: module(),
          epoch: pos_integer() | nil,
          current_epoch: pos_integer() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{actuator: actuator, command: command, epoch: nil}) do
    "Actuator #{inspect(actuator)} refused #{inspect(command)}: " <>
      "command carries no arm epoch"
  end

  def message(%{actuator: actuator, command: command, epoch: epoch, current_epoch: current}) do
    "Actuator #{inspect(actuator)} refused #{inspect(command)}: " <>
      "command was stamped for arm epoch #{epoch}, robot is now on #{inspect(current)}"
  end
end
