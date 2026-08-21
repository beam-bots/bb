# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.RecordingActuator do
  @moduledoc """
  Actuator that forwards every received command message to a test process.

  Before starting the robot, put the recipient pid into persistent term:

      :persistent_term.put({BB.Test.RecordingActuator, MyRobot}, self())

  The actuator looks up the recipient at init and sends
  `{:received, :command, message}` for each command, whichever transport
  delivered it - a driver can't tell, and shouldn't need to.

  It deliberately does not subscribe to its own command topic: the framework
  owns that subscription, and a double that subscribed for itself would pass
  the tests whether or not the framework did its job.

  Pass `subscribe_to:` to have it subscribe to some other topic. Messages
  arriving there are forwarded as `{:received, :info, message}`, which is how
  the tests check that the server leaves a driver's own subscriptions alone.

  Pass `delay_ms:` to have it block for that long before answering a command,
  standing in for a driver that waits on slow hardware.
  """
  use BB.Actuator,
    options_schema: [
      subscribe_to: [
        type: {:list, :atom},
        required: false,
        doc: "An additional pubsub topic for the actuator to subscribe to itself"
      ],
      delay_ms: [
        type: :non_neg_integer,
        required: false,
        default: 0,
        doc: "How long to block before answering a command"
      ]
    ]

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def capabilities(_opts), do: [:position_feedback]

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    recipient = :persistent_term.get({__MODULE__, bb.robot}, nil)

    case Keyword.get(opts, :subscribe_to) do
      nil -> :ok
      topic -> BB.subscribe(bb.robot, topic)
    end

    {:ok, %{recipient: recipient, delay_ms: Keyword.fetch!(opts, :delay_ms)}}
  end

  @impl BB.Actuator
  def handle_command(%BB.Message{} = message, state) do
    Process.sleep(state.delay_ms)
    forward(state.recipient, :command, message)
    {:noreply, state}
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %BB.Message{} = message}, state) do
    forward(state.recipient, :info, message)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  defp forward(nil, _kind, _message), do: :ok
  defp forward(pid, kind, message), do: send(pid, {:received, kind, message})
end
