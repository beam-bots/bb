# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Collision do
  @moduledoc """
  Collision information
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            origin: nil,
            geometry: nil,
            name: nil

  alias Kinetix.Dsl.{Box, Cylinder, Mesh, Origin, Sphere}
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          origin: nil | Origin.t(),
          geometry: nil | Box.t() | Cylinder.t() | Sphere.t() | Mesh.t(),
          name: atom
        }
end
