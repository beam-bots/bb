# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.Commands do
  @moduledoc false

  alias BB.Message
  alias BB.Safety

  @doc """
  Deliver a hand-built command to an actuator by cast.

  Keeps the shape of the private `{:command, message, reply_to}` tuple in one
  place, rather than in every test that exercises the cast transport.

  `reply_to` is the process `BB.Actuator.Server` messages when it refuses the
  command, and `nil` - the default - means nobody, which is what
  `BB.Actuator`'s send functions pass unless asked for `reply_on_reject?`.
  """
  @spec cast(module(), atom(), Message.t(), pid() | nil) :: :ok
  def cast(robot, actuator_name, %Message{} = message, reply_to \\ nil) do
    BB.cast(robot, actuator_name, {:command, message, reply_to})
  end

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
