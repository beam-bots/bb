# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.Timeout do
  @moduledoc """
  State transition timed out.

  Raised when a state transition does not complete within the
  expected time.
  """
  use BB.Error,
    class: :state,
    fields: [:from_state, :to_state, :timeout_ms]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{from_state: from, to_state: to, timeout_ms: timeout}) do
    "State transition timeout: #{inspect(from)} -> #{inspect(to)} " <>
      "did not complete within #{timeout}ms"
  end
end
