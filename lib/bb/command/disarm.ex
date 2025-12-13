# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.Disarm do
  @moduledoc """
  Standard command handler for disarming a robot.

  When executed from the `:idle` state, this command disarms the robot
  via `BB.Safety.Controller`, which calls all registered `BB.Safety.disarm/1`
  callbacks to ensure hardware is made safe.

  ## Usage

  Add to your robot's command definitions:

      commands do
        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle]
        end
      end

  Then execute:

      {:ok, task} = MyRobot.disarm()
      {:ok, :disarmed} = Task.await(task)

  """
  @behaviour BB.Command

  @impl true
  def handle_command(_goal, context) do
    case BB.Safety.disarm(context.robot_module) do
      :ok -> {:ok, :disarmed}
      {:error, reason} -> {:error, reason}
    end
  end
end
