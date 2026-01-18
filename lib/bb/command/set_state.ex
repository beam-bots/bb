# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.SetState do
  @moduledoc """
  Reusable command handler for transitioning between operational states.

  Use this handler to define simple state transition commands without
  implementing a custom handler. The target state is specified in the
  handler options.

  ## Usage

      commands do
        command :enter_recording do
          handler {BB.Command.SetState, to: :recording}
          allowed_states [:idle]
        end

        command :exit_recording do
          handler {BB.Command.SetState, to: :idle}
          allowed_states [:recording]
        end
      end

  Then execute:

      {:ok, cmd} = MyRobot.enter_recording()
      {:ok, :recording} = BB.Command.await(cmd)
      BB.Robot.Runtime.state(MyRobot)  # => :recording

  ## Handler Options

  - `:to` (required) - The target state to transition to. Must be defined
    in the robot's `states` section.

  """
  use BB.Command,
    options_schema: [
      to: [
        type: :atom,
        required: true,
        doc: "The target state to transition to"
      ]
    ]

  @impl BB.Command
  def handle_command(_goal, _context, state) do
    target_state = state.to
    {:stop, :normal, %{state | result: {:ok, target_state}, next_state: target_state}}
  end

  @impl BB.Command
  def result(%{result: {:ok, value}, next_state: next_state}) do
    {:ok, value, next_state: next_state}
  end

  def result(%{result: result}), do: result
end
