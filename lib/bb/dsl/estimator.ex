# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Estimator do
  @moduledoc """
  A state estimator nested inside a `sensor` (single-input form, frame
  inherited from the sensor) or a `link` (cross-sensor form, frame = link).

  See `BB.Estimator` for the behaviour contract and `BB.Estimator.Server`
  for runtime semantics.
  """

  alias BB.Dsl.Estimator.Input
  alias BB.Dsl.Estimator.Output
  alias Spark.Dsl.Entity

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            child_spec: nil,
            inputs: [],
            outputs: [],
            sync_tolerance: nil

  @type child_spec :: module | {module, Keyword.t()}

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          child_spec: child_spec,
          inputs: [Input.t()],
          outputs: [Output.t()],
          sync_tolerance: nil | Localize.Unit.t()
        }
end
