# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.MockController do
  @moduledoc """
  Minimal mock controller for testing.
  """
  use BB.Controller, options_schema: []

  @impl BB.Controller
  def init(opts) do
    {:ok, %{bb: opts[:bb]}}
  end

  @impl BB.Controller
  def handle_call(_request, _from, state) do
    {:reply, :ok, state}
  end

  @impl BB.Controller
  def handle_cast(_request, state) do
    {:noreply, state}
  end
end
