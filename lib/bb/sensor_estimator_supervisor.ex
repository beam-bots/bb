# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.SensorEstimatorSupervisor do
  @moduledoc """
  Supervisor for a sensor and the estimators nested inside it.

  Strategy is `:rest_for_one`, with the sensor first: when the sensor dies its
  estimators are stopped and started again after it, so each one subscribes to
  whatever the new sensor provides rather than racing it. An estimator dying
  on its own restarts only itself.

  That ordering is what makes a source that dies with its sensor - a camera's
  hub, say - safe to subscribe to in an estimator's `init/1`. Under a
  `:one_for_one` parent an estimator could be restarted before its sensor was,
  find nothing to subscribe to, and spend the parent's restart budget failing.

  The cost is that a sensor-nested estimator always loses its state when its
  sensor crashes, whatever its source.

  Its restart budget is configurable via the `sensor_max_restarts` and
  `sensor_max_seconds` settings on the robot DSL. A sensor that keeps crashing
  spends this budget before its parent sees a failure, so a lower
  `sensor_max_restarts` escalates a flapping sensor sooner.

  A sensor with no nested estimators gets no supervisor of its own; it stays a
  direct child of its link, joint or robot-level sensor supervisor.
  """

  alias BB.Dsl.Info
  alias BB.Estimator.Wiring

  @doc """
  The child spec for `sensor`, declared at `path`: this supervisor if the
  sensor has nested estimators, otherwise the sensor's own.

  `path` is the link or joint path the sensor sits under, or `[]` for a
  robot-level sensor.
  """
  @spec child_spec_for(module, BB.Dsl.Sensor.t(), [atom], Keyword.t()) ::
          Supervisor.child_spec()
  def child_spec_for(robot_module, %{estimators: []} = sensor, path, opts) do
    sensor_child_spec(robot_module, sensor, path, opts)
  end

  def child_spec_for(robot_module, sensor, path, opts) do
    %{
      id: sensor.name,
      start: {__MODULE__, :start_link, [{robot_module, sensor, path, opts}]},
      type: :supervisor
    }
  end

  @doc """
  Starts the supervisor for `sensor` and its estimators.
  """
  @spec start_link({module, BB.Dsl.Sensor.t(), [atom], Keyword.t()}) ::
          Supervisor.on_start()
  def start_link({robot_module, sensor, path, opts}) do
    settings = Info.settings(robot_module)
    sup_mod = settings.supervisor_module || Supervisor

    sensor_path = [:sensor | path ++ [sensor.name]]

    estimator_children =
      Enum.map(sensor.estimators, fn estimator ->
        Wiring.sensor_nested_child_spec(
          robot_module,
          estimator,
          sensor_path,
          sensor.child_spec,
          opts
        )
      end)

    children = [sensor_child_spec(robot_module, sensor, path, opts) | estimator_children]

    sup_mod.start_link(children,
      strategy: :rest_for_one,
      max_restarts: settings.sensor_max_restarts,
      max_seconds: settings.sensor_max_seconds
    )
  end

  defp sensor_child_spec(robot_module, sensor, path, opts) do
    BB.Process.child_spec(robot_module, sensor.name, sensor.child_spec, path, :sensor, opts)
  end
end
