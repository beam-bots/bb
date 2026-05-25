# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Estimator.Input do
  @moduledoc """
  A declared input on a cross-sensor estimator.

  Inputs are only used by link-nested estimators. Sensor-nested estimators
  consume their parent sensor's output implicitly.

  Exactly one input on a multi-input estimator must be marked `driver?:
  true`. The driver's arrival triggers fan-in: the framework snapshots the
  most-recent non-driver input messages and dispatches them together. If
  any non-driver input is stale relative to the driver by more than the
  estimator's `sync_tolerance`, the dispatch is dropped.

  Single-input link-nested estimators omit `driver?:` (or set it to `true`
  on the sole input).
  """

  alias Spark.Dsl.Entity

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            path: nil,
            driver?: false

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          path: [atom],
          driver?: boolean
        }
end
