# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.SensorSupervisor do
  @moduledoc """
  Supervisor for robot-level sensors.

  Groups all sensors defined in the `sensors` section under a single
  supervisor for fault isolation. A flapping sensor won't exhaust
  the root supervisor's restart budget.
  """

  use Supervisor

  alias BB.Dsl.Info
  alias BB.Estimator.Wiring

  def start_link({robot_module, opts}) do
    Supervisor.start_link(__MODULE__, {robot_module, opts},
      name: BB.Process.via(robot_module, __MODULE__)
    )
  end

  @impl true
  def init({robot_module, opts}) do
    sensors = Info.sensors(robot_module)

    sensor_children =
      Enum.map(sensors, fn sensor ->
        BB.Process.child_spec(robot_module, sensor.name, sensor.child_spec, [], :sensor, opts)
      end)

    estimator_children =
      Enum.flat_map(sensors, fn sensor ->
        sensor_path = [:sensor, sensor.name]

        Enum.map(sensor.estimators, fn estimator ->
          Wiring.sensor_nested_child_spec(
            robot_module,
            estimator,
            sensor_path,
            sensor.name,
            opts
          )
        end)
      end)

    Supervisor.init(sensor_children ++ estimator_children, strategy: :one_for_one)
  end
end
