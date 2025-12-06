# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Supervisor do
  @moduledoc """
  Root supervisor for a BB robot.

  Builds a supervision tree that mirrors the robot topology for fault isolation.
  A crash in an actuator at the end of a limb only affects that limb's subtree.

  ## Supervision Tree Structure

  ```
  BB.Supervisor (root, :one_for_one)
  ├── Registry (named {MyRobot, :registry})
  ├── PubSub Registry (named {MyRobot, :pubsub})
  ├── Task.Supervisor (for command execution tasks)
  ├── Runtime (robot state, state machine, command execution)
  ├── BB.SensorSupervisor (:one_for_one)
  │   └── RobotSensor1, RobotSensor2...
  ├── BB.ControllerSupervisor (:one_for_one)
  │   └── Controller1, Controller2...
  ├── BB.BridgeSupervisor (:one_for_one)
  │   └── MavlinkBridge, PhoenixBridge...
  └── BB.LinkSupervisor(:base_link, :one_for_one)
      ├── LinkSensor (link sensors)
      └── BB.JointSupervisor(:shoulder, :one_for_one)
          ├── JointSensor
          ├── JointActuator
          └── BB.LinkSupervisor(:arm, :one_for_one)
              └── ...
  ```

  Each subsystem supervisor (sensors, controllers, bridges) has its own restart
  budget, so a flapping process in one won't exhaust the root supervisor's
  budget and bring down the entire robot.
  """

  alias BB.Dsl.{Info, Link}

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
         name: BB.Process.registry_name(robot_module)
       )}

    pubsub_child =
      {settings.registry_module,
       Keyword.merge(settings.registry_options,
         keys: :duplicate,
         name: BB.PubSub.registry_name(robot_module)
       )}

    # Task supervisor for command execution
    task_supervisor_child =
      {Task.Supervisor, name: BB.Process.via(robot_module, BB.TaskSupervisor)}

    # Runtime manages robot state, state machine, and command execution
    runtime_child = {BB.Robot.Runtime, {robot_module, opts}}

    # Subsystem supervisors for fault isolation
    sensor_supervisor_child = {BB.SensorSupervisor, {robot_module, opts}}
    controller_supervisor_child = {BB.ControllerSupervisor, {robot_module, opts}}
    bridge_supervisor_child = {BB.BridgeSupervisor, {robot_module, opts}}

    # Links remain as entities at robot level
    link_children =
      robot_module
      |> Info.topology()
      |> Enum.filter(&is_struct(&1, Link))
      |> Enum.map(fn link ->
        {BB.LinkSupervisor, {robot_module, link, [], opts}}
      end)

    [
      registry_child,
      pubsub_child,
      task_supervisor_child,
      runtime_child,
      sensor_supervisor_child,
      controller_supervisor_child,
      bridge_supervisor_child
    ] ++ link_children
  end
end
