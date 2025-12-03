# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.Joint do
  @moduledoc """
  A joint in the robot topology chain.
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            type: nil,
            origin: nil,
            axis: nil,
            link: nil,
            dynamics: nil,
            limit: nil,
            sensors: [],
            actuators: []

  alias Kinetix.Dsl.{Actuator, Axis, Dynamics, Limit, Link, Origin, Sensor}
  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          type: :revolute | :continuous | :prismatic | :fixed | :floating | :planar,
          origin: nil | Origin.t(),
          axis: nil | Axis.t(),
          link: Link.t(),
          dynamics: nil | Dynamics.t(),
          limit: nil | Limit.t(),
          sensors: [Sensor.t()],
          actuators: [Actuator.t()]
        }
end
