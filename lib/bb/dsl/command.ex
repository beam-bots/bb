# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Command do
  @moduledoc """
  A command that can be executed on the robot.

  Commands follow the Goal → Feedback → Result pattern, supporting:
  - Arguments with types and defaults
  - State machine integration via `allowed_states`
  - Configurable timeout
  - A handler module implementing the `BB.Command` behaviour
  """

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            handler: nil,
            timeout: :infinity,
            allowed_states: [:idle],
            category: nil,
            arguments: []

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          handler: module,
          timeout: timeout,
          allowed_states: [atom],
          category: atom | nil,
          arguments: [BB.Dsl.Command.Argument.t()]
        }
end
