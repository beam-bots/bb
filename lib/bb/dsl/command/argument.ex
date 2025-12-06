# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Command.Argument do
  @moduledoc """
  An argument for a command.

  Arguments define the parameters that can be passed when executing a command.
  """

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            type: nil,
            required: false,
            default: nil,
            doc: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          type: atom | module,
          required: boolean,
          default: any,
          doc: String.t() | nil
        }
end
