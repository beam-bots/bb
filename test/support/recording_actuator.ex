# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.RecordingActuator do
  @moduledoc """
  Actuator that forwards every received command message to a test process.

  Before starting the robot, put the recipient pid into persistent term:

      :persistent_term.put({BB.Test.RecordingActuator, MyRobot}, self())

  The actuator looks up the recipient at init and sends
  `{:received, kind, message}` for each incoming command. `kind` is one of
  `:info`, `:cast`, or `:call`.
  """
  use BB.Actuator, options_schema: []

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    recipient = :persistent_term.get({__MODULE__, bb.robot}, nil)
    BB.subscribe(bb.robot, [:actuator | bb.path])
    {:ok, %{recipient: recipient}}
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %BB.Message{} = message}, state) do
    forward(state.recipient, :info, message)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  @impl BB.Actuator
  def handle_cast({:command, %BB.Message{} = message}, state) do
    forward(state.recipient, :cast, message)
    {:noreply, state}
  end

  @impl BB.Actuator
  def handle_call({:command, %BB.Message{} = message}, _from, state) do
    forward(state.recipient, :call, message)
    {:reply, {:ok, :accepted}, state}
  end

  defp forward(nil, _kind, _message), do: :ok
  defp forward(pid, kind, message), do: send(pid, {:received, kind, message})
end
