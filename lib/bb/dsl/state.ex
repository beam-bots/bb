# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.State do
  @moduledoc """
  A custom operational state for the robot.

  States define the operational context the robot can be in (beyond the
  built-in `:idle`). Commands specify which states they can run in via
  `allowed_states`, and can transition the robot to new states via
  `next_state:` in their result.

  The `:idle` state is always implicitly available and is the default
  initial state.
  """

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            doc: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          doc: String.t() | nil
        }
end
