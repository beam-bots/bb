# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Param do
  @moduledoc "A runtime-adjustable parameter."

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            type: nil,
            default: nil,
            min: nil,
            max: nil,
            doc: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          type: atom | {:unit, atom},
          default: term,
          min: number | nil,
          max: number | nil,
          doc: String.t() | nil
        }
end
