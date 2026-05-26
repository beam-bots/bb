# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Estimator.StaleInput do
  @moduledoc """
  An input message arrived too late to be useful.

  Raised when an estimator's `latency_budget` (Phase 2) is exceeded
  between an input's `monotonic_time` and the moment it is dispatched,
  or when a non-driver input falls outside the configured
  `sync_tolerance` of the driver and the algorithm has opted to surface
  the failure rather than silently drop.
  """

  use BB.Error, class: :state, fields: [:input_path, :age_ms, :budget_ms]

  @type t :: %__MODULE__{
          input_path: [atom()],
          age_ms: number(),
          budget_ms: number()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :warning
  end

  def message(%{input_path: path, age_ms: age, budget_ms: budget}) do
    "stale input at #{inspect(path)}: age #{age}ms exceeds budget #{budget}ms"
  end
end
