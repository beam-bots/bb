# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Estimator.SyncMiss do
  @moduledoc """
  A driver input arrived but a paired non-driver input was older than
  the estimator's `sync_tolerance`.

  Normally the framework drops these dispatches silently (with
  `[:bb, :estimator, :dropped]` telemetry, reason `:sync_miss`). This
  error type exists for algorithms or supervisors that want to surface
  the miss as a structured value instead.
  """

  use BB.Error,
    class: :state,
    fields: [:driver_path, :input_path, :gap_ms, :tolerance_ms]

  @type t :: %__MODULE__{
          driver_path: [atom()],
          input_path: [atom()],
          gap_ms: number(),
          tolerance_ms: number()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :warning
  end

  def message(%{driver_path: driver, input_path: input, gap_ms: gap, tolerance_ms: tol}) do
    "sync miss: driver #{inspect(driver)} vs input #{inspect(input)} — gap #{gap}ms exceeds tolerance #{tol}ms"
  end
end
