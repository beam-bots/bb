# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Limit do
  @moduledoc """
  Joint limits
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            lower: nil,
            upper: nil,
            effort: nil,
            velocity: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          lower: nil | Cldr.Unit.t(),
          upper: nil | Cldr.Unit.t(),
          effort: Cldr.Unit.t(),
          velocity: Cldr.Unit.t()
        }
end
