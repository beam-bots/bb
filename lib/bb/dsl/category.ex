# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Category do
  @moduledoc """
  A command category for grouping commands with concurrent execution limits.

  Categories define logical groups of commands (e.g., `:motion`, `:sensing`,
  `:auxiliary`) with configurable concurrency limits. Commands in different
  categories can run concurrently, while commands in the same category are
  limited to the category's `concurrency_limit`.

  The `:default` category is always implicitly available with a concurrency
  limit of 1, matching the current single-command behaviour.
  """

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            doc: nil,
            concurrency_limit: 1

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          doc: String.t() | nil,
          concurrency_limit: pos_integer()
        }
end
