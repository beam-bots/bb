# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.Commands do
  @moduledoc false

  alias BB.Message
  alias BB.Safety

  @doc """
  Stamp a hand-built command with the robot's current arm epoch.

  `BB.Actuator`'s send functions do this themselves; a test that builds a
  command and delivers it straight to an actuator has to do it explicitly or
  the actuator refuses the command as unstamped.

  Raises if the robot is not armed, since a test that meant to exercise the
  pipeline has already gone wrong by that point.
  """
  @spec stamp(module(), Message.t()) :: Message.t()
  def stamp(robot, %Message{} = message) do
    {:ok, epoch} = Safety.epoch(robot)
    %{message | arm_epoch: epoch}
  end
end
