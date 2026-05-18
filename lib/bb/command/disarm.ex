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

  ## Implicit `disarm: true`

  When this module is the handler, the DSL implicitly sets `disarm: true`
  on the command, which means `BB.Safety.disarm/2` will route through
  this command rather than flipping safety state directly. To insert
  pre-disarm work (e.g. moving to a home position) without losing the
  safety-API routing, write a custom handler that calls
  `BB.Safety.Controller.disarm/2` directly and flag it with
  `disarm true`. If the custom handler returns an error before reaching
  the controller, the safety system parks the robot in `:error` —
  `force_disarm/1` is required to recover.

  """
  use BB.Command

  alias BB.Safety.Controller

  @impl BB.Command
  def handle_command(_goal, context, state) do
    # Use Controller.disarm/2 directly to avoid re-entering BB.Safety.disarm/2's
    # command-routing layer (which would dispatch this same command and loop).
    case Controller.disarm(context.robot_module) do
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
