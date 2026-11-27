# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.RecordingEstimator do
  @moduledoc """
  Estimator that forwards every input it is given to a test process.

  Before starting the robot, put the recipient pid into persistent term:

      :persistent_term.put({BB.Test.RecordingEstimator, MyRobot}, self())

  The estimator looks up the recipient at init and sends
  `{:received, :input, input}` for each call to `handle_input/2`, whatever its
  source delivered - a `%BB.Message{}`, a multi-input map, or anything else.

  Pass `gated: true` to have it hold each input until the test sends the
  estimator's process `:release`, standing in for an estimator doing slow
  work. It sends `{:received, :holding, input}` as it starts waiting, so a
  test can tell the input has arrived without guessing how long that takes.
  Every held input must be released, or the robot can't shut down.
  """
  use BB.Estimator,
    options_schema: [
      gated: [
        type: :boolean,
        required: false,
        default: false,
        doc: "Hold each input until the estimator is sent `:release`"
      ]
    ]

  @impl BB.Estimator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    recipient = :persistent_term.get({__MODULE__, bb.robot}, nil)
    {:ok, %{recipient: recipient, gated: Keyword.fetch!(opts, :gated)}}
  end

  @impl BB.Estimator
  def handle_input(input, state) do
    if state.gated, do: hold(state.recipient, input)
    forward(state.recipient, :input, input)
    {:noreply, state}
  end

  defp hold(recipient, input) do
    forward(recipient, :holding, input)

    receive do
      :release -> :ok
    end
  end

  defp forward(nil, _kind, _input), do: :ok
  defp forward(pid, kind, input), do: send(pid, {:received, kind, input})
end
