# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Transmission do
  @moduledoc """
  Mechanical transmission between a joint and its actuator(s).

  Captures the relationship between joint-space command and motor-space
  command — gear reduction, zero-offset, and polarity. The URDF equivalent
  is `<transmission>`.
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            reduction: 1.0,
            offset: nil,
            reversed?: false

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          reduction: number,
          offset: nil | Localize.Unit.t(),
          reversed?: boolean
        }
end
