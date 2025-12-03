# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Command.Execution do
  @moduledoc """
  Tracks the state of a command execution.
  """

  defstruct [
    :id,
    :command_name,
    :goal,
    :caller,
    :status,
    :started_at,
    :handler_state
  ]

  @type status ::
          :pending
          | :accepted
          | :executing
          | :canceling
          | :succeeded
          | :aborted
          | :canceled
          | :rejected

  @type t :: %__MODULE__{
          id: reference(),
          command_name: atom(),
          goal: map(),
          caller: GenServer.from(),
          status: status(),
          started_at: integer(),
          handler_state: term()
        }

  @doc """
  Create a new execution for a goal.
  """
  @spec new(atom(), map(), GenServer.from()) :: t()
  def new(command_name, goal, caller) do
    %__MODULE__{
      id: make_ref(),
      command_name: command_name,
      goal: goal,
      caller: caller,
      status: :pending,
      started_at: System.monotonic_time(:nanosecond),
      handler_state: nil
    }
  end

  @doc """
  Check if the execution is in a terminal state.
  """
  @spec terminal?(t()) :: boolean()
  def terminal?(%__MODULE__{status: status}) do
    status in [:succeeded, :aborted, :canceled, :rejected]
  end
end
