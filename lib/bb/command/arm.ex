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

      {:ok, cmd} = MyRobot.arm()
      {:ok, :armed} = BB.Command.await(cmd)

  """
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, context, state) do
    case BB.Safety.arm(context.robot_module) do
      :ok ->
        {:stop, :normal, %{state | result: {:ok, :armed}, next_state: :idle}}

      {:error, reason} ->
        {:stop, :normal, %{state | result: {:error, reason}}}
    end
  end

  @impl BB.Command
  def result(%{result: {:ok, value}, next_state: next_state}) do
    {:ok, value, next_state: next_state}
  end

  def result(%{result: result}), do: result
end
