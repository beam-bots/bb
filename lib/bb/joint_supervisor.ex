# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.JointSupervisor do
  @moduledoc """
  Supervisor for a joint and its child link.

  Supervises:
  - Joint sensors
  - Joint actuators
  - Child link supervisor (if joint has a child link)
  """

  alias BB.Dsl.Info

  @doc """
  Starts the joint supervisor.

  ## Arguments

  - `robot_module` - The robot module (e.g., `MyRobot`)
  - `joint` - The `BB.Dsl.Joint` struct
  - `path` - The path to this joint (e.g., `[:base_link]`)
  - `opts` - Options passed through to child processes
  """
  @spec start_link({module, BB.Dsl.Joint.t(), [atom], Keyword.t()}) ::
          Supervisor.on_start()
  def start_link({robot_module, joint, path, opts}) do
    settings = Info.settings(robot_module)
    sup_mod = settings.supervisor_module || Supervisor

    children = build_children(robot_module, joint, path, opts)
    sup_mod.start_link(children, strategy: :one_for_one)
  end

  @doc false
  def child_spec({robot_module, joint, path, opts}) do
    %{
      id: joint.name,
      start: {__MODULE__, :start_link, [{robot_module, joint, path, opts}]},
      type: :supervisor
    }
  end

  defp build_children(robot_module, joint, path, opts) do
    joint_path = path ++ [joint.name]
    simulation_mode = Keyword.get(opts, :simulation)

    sensor_children =
      Enum.map(joint.sensors, fn sensor ->
        BB.Process.child_spec(
          robot_module,
          sensor.name,
          sensor.child_spec,
          joint_path,
          :sensor,
          opts
        )
      end)

    actuator_children =
      Enum.map(joint.actuators, fn actuator ->
        BB.Process.child_spec(
          robot_module,
          actuator.name,
          actuator.child_spec,
          joint_path,
          :actuator,
          opts
        )
      end)

    auto_position_estimators =
      if simulation_mode do
        build_auto_position_estimators(robot_module, joint, joint_path, opts)
      else
        []
      end

    link_child =
      if joint.link do
        [{BB.LinkSupervisor, {robot_module, joint.link, joint_path, opts}}]
      else
        []
      end

    sensor_children ++ auto_position_estimators ++ actuator_children ++ link_child
  end

  defp build_auto_position_estimators(robot_module, joint, joint_path, opts) do
    joint.actuators
    |> Enum.filter(fn actuator ->
      not has_position_sensor_for_actuator?(joint.sensors, actuator.name)
    end)
    |> Enum.map(fn actuator ->
      sensor_name = :"#{actuator.name}_position_estimator"

      BB.Process.child_spec(
        robot_module,
        sensor_name,
        {BB.Sensor.OpenLoopPositionEstimator, actuator: actuator.name},
        joint_path,
        :sensor,
        opts
      )
    end)
  end

  defp has_position_sensor_for_actuator?(sensors, actuator_name) do
    Enum.any?(sensors, fn sensor ->
      case sensor.child_spec do
        {BB.Sensor.OpenLoopPositionEstimator, sensor_opts} ->
          Keyword.get(sensor_opts, :actuator) == actuator_name

        _ ->
          false
      end
    end)
  end
end
