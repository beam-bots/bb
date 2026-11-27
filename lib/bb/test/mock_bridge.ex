# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.MockBridge do
  @moduledoc """
  Test double: a bridge that accepts every parameter change and every call or
  cast, and does nothing with them.

  For a robot fixture that needs a bridge declared without the test being
  about what it bridges to. For one that records what it was asked, see
  `BB.Test.ParameterBridge`.
  """
  use BB.Bridge, options_schema: []

  @impl GenServer
  def init(opts) do
    {:ok, %{bb: opts[:bb]}}
  end

  @impl BB.Bridge
  def handle_change(_robot, _changed, state) do
    {:ok, state}
  end

  @impl GenServer
  def handle_call(_request, _from, state) do
    {:reply, :ok, state}
  end

  @impl GenServer
  def handle_cast(_request, state) do
    {:noreply, state}
  end
end
