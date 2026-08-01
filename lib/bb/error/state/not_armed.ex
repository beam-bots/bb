# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.NotArmed do
  @moduledoc """
  Command refused because the robot is not armed.

  `BB.Actuator.Server` checks `BB.Safety.armed?/1` before handing a command
  to a driver, so no actuator can drive hardware while the robot is disarmed.
  Stop commands are exempt and never produce this error.

  This is a `:state` error rather than a `:safety` one: the safety system
  working as intended is not a safety violation.
  """
  use BB.Error,
    class: :state,
    fields: [:robot, :actuator, :command]

  @type t :: %__MODULE__{
          robot: module() | nil,
          actuator: atom(),
          command: module()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{actuator: actuator, command: command}) do
    "Actuator #{inspect(actuator)} refused #{inspect(command)}: robot is not armed"
  end
end
