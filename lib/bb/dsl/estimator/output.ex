# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Estimator.Output do
  @moduledoc """
  A declared output on an estimator.

  Most estimators emit a single output via the conventional `:out` name and
  do not need an explicit `output` block - the framework synthesises one
  pointing at the estimator's natural path. Multi-output estimators
  declare each output explicitly so subscribers can address them by name.

  When set, `path:` overrides the auto-derived output path. Useful for
  routing a derived output onto a foreign topic.
  """

  alias Spark.Dsl.Entity

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            path: nil

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          path: nil | [atom]
        }
end
