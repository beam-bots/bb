# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.Source.PubSub do
  @moduledoc """
  The default `BB.Estimator.Source`: every `%BB.Message{}` published at a path.

  PubSub has no notion of demand, so `request/1` does nothing and an estimator
  slower than its input simply accumulates messages in its mailbox.
  """

  @behaviour BB.Estimator.Source

  alias BB.Message

  @impl BB.Estimator.Source
  def subscribe(robot, path, _args) do
    with {:ok, _pid} <- BB.PubSub.subscribe(robot, path) do
      {:ok, path}
    end
  end

  @impl BB.Estimator.Source
  def input_event(path, {:bb, path, %Message{} = message}),
    do: {:input, message, message.monotonic_time, message.node}

  def input_event(_path, _message), do: nil

  @impl BB.Estimator.Source
  def request(_path), do: :ok
end
