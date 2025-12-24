# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Sensor do
  @moduledoc "A sensor attached to the robot or a specific link."

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            child_spec: nil

  alias Spark.Dsl.Entity

  @type child_spec :: module | {module, Keyword.t()}

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          child_spec: child_spec
        }
end
