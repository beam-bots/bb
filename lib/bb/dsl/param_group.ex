# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ParamGroup do
  @moduledoc "A group of runtime-adjustable parameters."

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            doc: nil,
            params: [],
            groups: []

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          doc: String.t() | nil,
          params: [BB.Dsl.Param.t()],
          groups: [t()]
        }
end
