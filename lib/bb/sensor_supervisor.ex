# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.SensorSupervisor do
  @moduledoc """
  Supervisor for robot-level sensors.

  Groups all sensors defined in the `sensors` section under a single
  supervisor for fault isolation. A flapping sensor won't exhaust
  the root supervisor's restart budget. A sensor with nested estimators sits
  under its own `BB.SensorEstimatorSupervisor`.
  """

  use Supervisor

  alias BB.Dsl.Info
  alias BB.SensorEstimatorSupervisor

  def start_link({robot_module, opts}) do
    Supervisor.start_link(__MODULE__, {robot_module, opts},
      name: BB.Process.via(robot_module, __MODULE__)
    )
  end

  @impl true
  def init({robot_module, opts}) do
    children =
      robot_module
      |> Info.sensors()
      |> Enum.map(&SensorEstimatorSupervisor.child_spec_for(robot_module, &1, [], opts))

    Supervisor.init(children, strategy: :one_for_one)
  end
end
