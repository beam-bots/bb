defmodule Kinetix.Dsl.Inertia do
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
          ixx: Cldr.Unit.t(),
          iyy: Cldr.Unit.t(),
          izz: Cldr.Unit.t(),
          ixy: Cldr.Unit.t(),
          ixz: Cldr.Unit.t(),
          iyz: Cldr.Unit.t()
        }
end
