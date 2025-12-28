# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Controller do
  @moduledoc "A controller process at the robot level."

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            child_spec: nil,
            simulation: :omit

  alias Spark.Dsl.Entity

  @type child_spec :: module | {module, Keyword.t()}
  @type simulation_mode :: :omit | :mock | :start

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          child_spec: child_spec,
          simulation: simulation_mode
        }
end
