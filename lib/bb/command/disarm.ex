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

      {:ok, cmd} = MyRobot.disarm()
      {:ok, :disarmed} = BB.Command.await(cmd)

  """
  use BB.Command

  @impl BB.Command
  def handle_command(_goal, context, state) do
    case BB.Safety.disarm(context.robot_module) do
      :ok ->
        {:stop, :normal, %{state | result: {:ok, :disarmed}, next_state: :disarmed}}

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
