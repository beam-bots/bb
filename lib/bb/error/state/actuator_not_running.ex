# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.ActuatorNotRunning do
  @moduledoc """
  A command could not be delivered because its actuator is not running.

  The actuator is declared but has no process: its driver's `init/1` returned
  `:ignore`, it is between crashing and being restarted, or the robot's
  supervision tree is not up yet. Nothing received the command and nothing
  will act on it.

  `delivery: :pubsub` checks for the actuator before publishing, so in the
  usual case the command never appears on the actuator's topic either. An
  actuator that dies between that check and the call is reported the same
  way, by which point the command has been published.
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
    "Actuator #{inspect(actuator)} is not running, so #{inspect(command)} was not delivered"
  end
end
