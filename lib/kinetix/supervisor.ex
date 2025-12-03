# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Supervisor do
  @moduledoc """
  Root supervisor for a Kinetix robot.

  Builds a supervision tree that mirrors the robot topology for fault isolation.
  A crash in an actuator at the end of a limb only affects that limb's subtree.

  ## Supervision Tree Structure

  ```
  Kinetix.Supervisor (root, :one_for_one)
  ├── Registry (named {MyRobot, :registry})
  ├── PubSub Registry (named {MyRobot, :pubsub})
  ├── Task.Supervisor (for command execution tasks)
  ├── Runtime (robot state, state machine, command execution)
  ├── RobotSensor1 (robot-level sensors)
  ├── Controller1 (robot-level controllers)
  └── Kinetix.LinkSupervisor(:base_link, :one_for_one)
      ├── LinkSensor (link sensors)
      └── Kinetix.JointSupervisor(:shoulder, :one_for_one)
          ├── JointSensor
          ├── JointActuator
          └── Kinetix.LinkSupervisor(:arm, :one_for_one)
              └── ...
  ```
  """

  alias Kinetix.Dsl.{Info, Link}

  @doc """
  Starts the supervisor tree for a robot module.

  ## Options

  All options are passed through to sensor and actuator child processes.
  """
  @spec start_link(module, Keyword.t()) :: Supervisor.on_start()
  def start_link(robot_module, opts \\ []) do
    settings = Info.settings(robot_module)
    sup_mod = settings.supervisor_module || Supervisor

    children = build_children(robot_module, settings, opts)

    sup_mod.start_link(children, strategy: :one_for_one, name: robot_module)
  end

  defp build_children(robot_module, settings, opts) do
    registry_child =
      {settings.registry_module,
       Keyword.merge(settings.registry_options,
         keys: :unique,
         name: Kinetix.Process.registry_name(robot_module)
       )}

    pubsub_child =
      {settings.registry_module,
       Keyword.merge(settings.registry_options,
         keys: :duplicate,
         name: Kinetix.PubSub.registry_name(robot_module)
       )}

    # Task supervisor for command execution
    task_supervisor_child =
      {Task.Supervisor, name: Kinetix.Process.via(robot_module, Kinetix.TaskSupervisor)}

    # Runtime manages robot state, state machine, and command execution
    runtime_child = {Kinetix.Robot.Runtime, {robot_module, opts}}

    # Sensors from the robot_sensors section
    robot_sensor_children =
      robot_module
      |> Info.sensors()
      |> Enum.map(fn sensor ->
        Kinetix.Process.child_spec(robot_module, sensor.name, sensor.child_spec, [])
      end)

    # Controllers from the controllers section
    controller_children =
      robot_module
      |> Info.controllers()
      |> Enum.map(fn controller ->
        Kinetix.Process.child_spec(robot_module, controller.name, controller.child_spec, [])
      end)

    # Links remain as entities at robot level
    link_children =
      robot_module
      |> Info.topology()
      |> Enum.filter(&is_struct(&1, Link))
      |> Enum.map(fn link ->
        {Kinetix.LinkSupervisor, {robot_module, link, [], opts}}
      end)

    [registry_child, pubsub_child, task_supervisor_child, runtime_child] ++
      robot_sensor_children ++ controller_children ++ link_children
  end
end
