# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.LinkSupervisor do
  @moduledoc """
  Supervisor for a link and its joints.

  Supervises:
  - Link sensors
  - Joint supervisors for each joint attached to this link
  """

  alias BB.Dsl.Info
  alias BB.Estimator.Wiring

  @doc """
  Starts the link supervisor.

  ## Arguments

  - `robot_module` - The robot module (e.g., `MyRobot`)
  - `link` - The `BB.Dsl.Link` struct
  - `path` - The path to this link (e.g., `[]` for root, `[:base_link, :shoulder]` for nested)
  - `opts` - Options passed through to child processes
  """
  @spec start_link({module, BB.Dsl.Link.t(), [atom], Keyword.t()}) ::
          Supervisor.on_start()
  def start_link({robot_module, link, path, opts}) do
    settings = Info.settings(robot_module)
    sup_mod = settings.supervisor_module || Supervisor

    children = build_children(robot_module, link, path, opts)
    sup_mod.start_link(children, strategy: :one_for_one)
  end

  @doc false
  def child_spec({robot_module, link, path, opts}) do
    %{
      id: link.name,
      start: {__MODULE__, :start_link, [{robot_module, link, path, opts}]},
      type: :supervisor
    }
  end

  defp build_children(robot_module, link, path, opts) do
    link_path = path ++ [link.name]

    sensor_children =
      Enum.map(link.sensors, fn sensor ->
        BB.Process.child_spec(
          robot_module,
          sensor.name,
          sensor.child_spec,
          link_path,
          :sensor,
          opts
        )
      end)

    sensor_estimator_children =
      Enum.flat_map(link.sensors, fn sensor ->
        sensor_path = [:sensor | link_path ++ [sensor.name]]

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

    estimator_children =
      Enum.map(link.estimators, fn estimator ->
        Wiring.link_nested_child_spec(robot_module, estimator, link_path, opts)
      end)

    joint_children =
      Enum.map(link.joints, fn joint ->
        {BB.JointSupervisor, {robot_module, joint, link_path, opts}}
      end)

    sensor_children ++ sensor_estimator_children ++ estimator_children ++ joint_children
  end
end
