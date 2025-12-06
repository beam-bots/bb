# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.Arm do
  @moduledoc """
  Standard command handler for arming a robot.

  When executed from the `:disarmed` state, this command transitions the robot
  to `:idle`, making it ready to accept motion commands.

  ## Usage

  Add to your robot's command definitions:

      commands do
        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end
      end

  Then execute:

      {:ok, task} = MyRobot.arm()
      {:ok, :armed} = Task.await(task)

  """
  @behaviour BB.Command

  @impl true
  def handle_command(_goal, _context) do
    {:ok, :armed}
  end
end
