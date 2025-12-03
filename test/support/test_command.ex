# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Test.ImmediateSuccessCommand do
  @moduledoc false
  @behaviour Kinetix.Command

  @impl true
  def init(_opts), do: {:ok, %{}}

  @impl true
  def handle_goal(_goal, _robot_state, state), do: {:accept, state}

  @impl true
  def handle_execute(_robot_state, _runtime, state), do: {:succeeded, :done, state}

  @impl true
  def handle_cancel(_robot_state, _runtime, state), do: {:canceled, :stopped, state}

  @impl true
  def handle_info(_msg, _robot_state, _runtime, state), do: {:executing, state}
end

defmodule Kinetix.Test.AsyncCommand do
  @moduledoc false
  @behaviour Kinetix.Command

  @impl true
  def init(_opts), do: {:ok, %{caller: nil}}

  @impl true
  def handle_goal(%{notify: pid}, _robot_state, state) do
    {:accept, %{state | caller: pid}}
  end

  def handle_goal(_goal, _robot_state, state), do: {:accept, state}

  @impl true
  def handle_execute(_robot_state, _runtime, state) do
    if state.caller, do: send(state.caller, :executing)
    {:executing, state}
  end

  @impl true
  def handle_cancel(_robot_state, _runtime, state), do: {:canceled, :stopped, state}

  @impl true
  def handle_info(:complete, _robot_state, _runtime, state), do: {:succeeded, :completed, state}
  def handle_info(:abort, _robot_state, _runtime, state), do: {:aborted, :failed, state}
  def handle_info(_msg, _robot_state, _runtime, state), do: {:executing, state}
end

defmodule Kinetix.Test.RejectingCommand do
  @moduledoc false
  @behaviour Kinetix.Command

  @impl true
  def init(_opts), do: {:ok, %{}}

  @impl true
  def handle_goal(_goal, _robot_state, state), do: {:reject, :not_allowed, state}

  @impl true
  def handle_execute(_robot_state, _runtime, state), do: {:executing, state}

  @impl true
  def handle_cancel(_robot_state, _runtime, state), do: {:canceled, :stopped, state}

  @impl true
  def handle_info(_msg, _robot_state, _runtime, state), do: {:executing, state}
end
