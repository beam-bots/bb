defmodule Kinetix.Dsl.Inertial do
  @moduledoc """
  Inertial information.
  """
  defstruct __identifier__: nil, __spark_metadata__: nil, origin: nil, mass: nil, inertia: nil

  alias Kinetix.Dsl.{Inertia, Origin}
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          origin: nil | Origin.t(),
          mass: Cldr.Unit.t(),
          inertia: Inertia.t()
        }
end
