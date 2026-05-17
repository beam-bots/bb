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
          lower: nil | Localize.Unit.t(),
          upper: nil | Localize.Unit.t(),
          effort: Localize.Unit.t(),
          velocity: Localize.Unit.t(),
          acceleration: nil | Localize.Unit.t()
        }
end
