# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Axis do
  @moduledoc """
  An axis
  """
  import Kinetix.Unit

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            x: ~u(0 meter),
            y: ~u(0 meter),
            z: ~u(0 meter)

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          x: Cldr.Unit.t(),
          y: Cldr.Unit.t(),
          z: Cldr.Unit.t()
        }
end
