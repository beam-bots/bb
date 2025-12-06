# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Mesh do
  @moduledoc """
  A 3D model (mesh)
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            filename: nil,
            scale: 1

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          filename: String.t(),
          scale: number
        }
end
