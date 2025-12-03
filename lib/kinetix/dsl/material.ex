# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Material do
  @moduledoc """
  A material
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            color: nil,
            texture: nil,
            name: nil

  alias Kinetix.Dsl.{Color, Texture}
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          color: nil | Color.t(),
          texture: nil | Texture.t(),
          name: atom
        }
end
