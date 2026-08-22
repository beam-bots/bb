# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Inertia do
  @moduledoc """
  Inertial information.
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            ixx: nil,
            iyy: nil,
            izz: nil,
            ixy: nil,
            ixz: nil,
            iyz: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          ixx: BB.Unit.t(),
          iyy: BB.Unit.t(),
          izz: BB.Unit.t(),
          ixy: BB.Unit.t(),
          ixz: BB.Unit.t(),
          iyz: BB.Unit.t()
        }
end
