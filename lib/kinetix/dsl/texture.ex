defmodule Kinetix.Dsl.Texture do
  @moduledoc """
  A 2D texture
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            filename: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          filename: String.t()
        }
end
