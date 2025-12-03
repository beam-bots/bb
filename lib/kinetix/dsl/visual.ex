# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Visual do
  @moduledoc """
  A material
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            geometry: nil,
            material: nil,
            origin: nil

  alias Kinetix.Dsl.{Box, Cylinder, Material, Mesh, Origin, Sphere}
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          geometry: nil | Box.t() | Cylinder.t() | Sphere.t() | Mesh.t() | Material.t(),
          material: nil | Material.t(),
          origin: nil | Origin.t()
        }
end
