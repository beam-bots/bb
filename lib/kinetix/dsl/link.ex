# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Link do
  @moduledoc """
  A kinematic link aka a solid body in a kinematic chain.
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            collisions: [],
            joints: [],
            sensors: [],
            visual: nil,
            inertial: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          collisions: [Kinetix.Dsl.Collision.t()],
          joints: [Kinetix.Dsl.Joint.t()],
          sensors: [Kinetix.Dsl.Sensor.t()],
          visual: Kinetix.Dsl.Visual.t(),
          inertial: nil | Kinetix.Dsl.Inertial.t()
        }
end
