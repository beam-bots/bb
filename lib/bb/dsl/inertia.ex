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
          ixx: Localize.Unit.t(),
          iyy: Localize.Unit.t(),
          izz: Localize.Unit.t(),
          ixy: Localize.Unit.t(),
          ixz: Localize.Unit.t(),
          iyz: Localize.Unit.t()
        }
end
