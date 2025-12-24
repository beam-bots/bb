# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.Preempted do
  @moduledoc """
  Operation was preempted by another operation.

  Raised when an in-progress operation is cancelled because a
  higher-priority operation has taken over.
  """
  use BB.Error,
    class: :state,
    fields: [:preempted_operation, :preempting_operation]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{preempted_operation: preempted, preempting_operation: preempting}) do
    "Operation #{inspect(preempted)} was preempted by #{inspect(preempting)}"
  end
end
