# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Box do
  @moduledoc """
  A box geometry
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            x: nil,
            y: nil,
            z: nil

  alias Cldr.Unit
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          x: Unit.t(),
          y: Unit.t(),
          z: Unit.t()
        }
end
