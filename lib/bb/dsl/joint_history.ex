# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.JointHistory do
  @moduledoc """
  How much joint-state history a robot keeps, and how it is read back.

  See `BB.Robot.State.get_all_configurations_at/2`.

  The struct's defaults are the DSL's defaults: a robot that never declares
  `joint_history` gets `%BB.Dsl.JointHistory{}`, so the two must not drift.
  """
  import BB.Unit

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            enabled?: true,
            keep: ~u(10 second),
            interpolation?: true,
            max_gap: ~u(1 second)

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          enabled?: boolean(),
          keep: Localize.Unit.t(),
          interpolation?: boolean(),
          max_gap: Localize.Unit.t()
        }
end
