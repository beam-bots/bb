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

  ## Implicit `arm: true`

  When this module is the handler, the DSL implicitly sets `arm: true` on
  the command, which means `BB.Safety.arm/1` will route through this
  command rather than flipping safety state directly. To insert pre-arm
  work (e.g. moving to a home position) without losing the safety-API
  routing, write a custom handler that calls
  `BB.Safety.Controller.arm/1` directly and flag it with `arm true`.

  """
  use BB.Command

  alias BB.Dsl.Info
  alias BB.Safety.Controller

  @impl BB.Command
  def handle_command(_goal, context, state) do
    # Use Controller.arm/1 directly to avoid re-entering BB.Safety.arm/1's
    # command-routing layer (which would dispatch this same command and loop).
    case Controller.arm(context.robot_module) do
      :ok ->
        # Transition to the robot's configured initial operational state.
        # The state machine was sitting at initial_state while :disarmed; we
        # set next_state explicitly to that value so that armed observers see
        # the same operational state regardless of whether arm was invoked
        # via this command or directly via Controller.arm/1.
        next_state = Info.initial_state(context.robot_module)
        {:stop, :normal, %{state | result: {:ok, :armed}, next_state: next_state}}

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
