# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.Arm do
  @moduledoc """
  Standard command handler for arming a robot.

  When executed from the `:disarmed` state, this command arms the robot
  via `BB.Safety.Controller`, making it ready to accept motion commands.

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
  def handle_command(_goal, context) do
    case BB.Safety.arm(context.robot_module) do
      :ok -> {:ok, :armed, next_state: :idle}
      {:error, reason} -> {:error, reason}
    end
  end
end
