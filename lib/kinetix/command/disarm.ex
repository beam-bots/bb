# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Command.Disarm do
  @moduledoc """
  Standard command handler for disarming a robot.

  When executed from the `:idle` state, this command transitions the robot
  to `:disarmed`, preventing further motion commands until re-armed.

  ## Usage

  Add to your robot's command definitions:

      commands do
        command :disarm do
          handler Kinetix.Command.Disarm
          allowed_states [:idle]
        end
      end

  Then execute:

      {:ok, task} = MyRobot.disarm()
      {:ok, :disarmed} = Task.await(task)

  """
  @behaviour Kinetix.Command

  @impl true
  def handle_command(_goal, _context) do
    {:ok, :disarmed, next_state: :disarmed}
  end
end
