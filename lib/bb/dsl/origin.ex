# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Origin do
  @moduledoc """
  An origin location.
  """
  import BB.Unit

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            roll: ~u(0 degree),
            pitch: ~u(0 degree),
            yaw: ~u(0 degree),
            x: ~u(0 meter),
            y: ~u(0 meter),
            z: ~u(0 meter)

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          roll: Localize.Unit.t(),
          pitch: Localize.Unit.t(),
          yaw: Localize.Unit.t(),
          x: Localize.Unit.t(),
          y: Localize.Unit.t(),
          z: Localize.Unit.t()
        }
end
