# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Origin do
  @moduledoc """
  An origin location.
  """
  import Kinetix.Unit

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
          roll: Cldr.Unit.t(),
          pitch: Cldr.Unit.t(),
          yaw: Cldr.Unit.t(),
          x: Cldr.Unit.t(),
          y: Cldr.Unit.t(),
          z: Cldr.Unit.t()
        }
end
