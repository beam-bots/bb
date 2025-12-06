# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Cylinder do
  @moduledoc """
  A cylindrical geometry
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
