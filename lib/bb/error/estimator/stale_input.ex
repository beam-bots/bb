# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Estimator.StaleInput do
  @moduledoc """
  An input message arrived too late to be useful.

  The framework drops these envelopes itself, transitioning the estimator
  to `:degraded` with reason `:stale_input` and emitting
  `[:bb, :estimator, :dropped]` telemetry. This error type exists for
  algorithms or supervisors that want to surface the age overrun as a
  structured value instead — for instance when an input exceeded its
  `max_input_age`.
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
