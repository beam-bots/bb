# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Capsule do
  @moduledoc """
  A capsule geometry (cylinder with hemispherical caps).

  Capsules are defined by a radius and height. The height is the distance
  between the centres of the two hemispherical caps (i.e., the length of
  the cylindrical portion). The total extent is `height + 2 * radius`.

  Capsules are commonly used for collision detection because they have
  simpler intersection algorithms than cylinders and better approximate
  robot limbs.
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            radius: nil,
            height: nil

  alias Cldr.Unit
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          radius: Unit.t(),
          height: Unit.t()
        }
end
