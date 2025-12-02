defmodule Kinetix.Dsl.Dynamics do
  @moduledoc """
  Specifies physical properties of the joint. These values are used to specify modeling properties of the joint, particularly useful for simulation
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            damping: nil,
            friction: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          damping: nil | Cldr.Unit.t(),
          friction: nil | Cldr.Unit.t()
        }
end
