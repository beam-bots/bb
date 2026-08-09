# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Limit do
  @moduledoc """
  Joint limits
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            lower: nil,
            upper: nil,
            effort: nil,
            velocity: nil,
            acceleration: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          lower: nil | BB.Unit.t(),
          upper: nil | BB.Unit.t(),
          effort: BB.Unit.t(),
          velocity: BB.Unit.t(),
          acceleration: nil | BB.Unit.t()
        }
end
