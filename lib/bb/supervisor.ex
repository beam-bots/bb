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
  ├── Task.Supervisor (for general async tasks)
  ├── DynamicSupervisor (for command GenServers, temporary restart)
  ├── Runtime (robot state, state machine, command execution)
  ├── BB.BridgeSupervisor (:one_for_one)
  │   └── MavlinkBridge, PhoenixBridge...
  └── BB.TopologySupervisor (:one_for_one)
      ├── BB.SensorSupervisor (:one_for_one)
      │   └── RobotSensor1, RobotSensor2...
      ├── BB.ControllerSupervisor (:one_for_one)
      │   └── Controller1, Controller2...
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
  budget and bring down the entire robot. The topology supervisor groups all
  hardware-facing subtrees so they share a restart budget; when that budget is
  exhausted the safety controller force-disarms the robot.
  """

  alias BB.Dsl.Info

  @doc """
  Starts the supervisor tree for a robot module.

  ## Options

    * `:params` - Initial parameter values as a nested keyword list matching
      the parameter group structure. Overrides DSL defaults and persisted values.

          BB.Supervisor.start_link(MyRobot, params: [
            motion: [max_speed: 5.0, acceleration: 2.0],
            debug_mode: true
          ])

    * `:simulation` - Simulation mode (`:kinematic` or `:external`). When set,
      actuators are replaced with simulated versions and controllers may be
      omitted.

  All options are also passed through to sensor, actuator, and controller
  child processes via the `:bb` key in their start options.
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

    # Task supervisor for general async tasks
    task_supervisor_child =
      {Task.Supervisor, name: BB.Process.via(robot_module, BB.TaskSupervisor)}

    # DynamicSupervisor for command GenServers (temporary, not restarted on crash)
    command_supervisor_child =
      {DynamicSupervisor,
       name: BB.Process.via(robot_module, BB.CommandSupervisor), strategy: :one_for_one}

    # Runtime manages robot state, state machine, and command execution
    runtime_child = {BB.Robot.Runtime, {robot_module, opts}}

    # External communication, not hardware - stays at root
    bridge_supervisor_child = {BB.BridgeSupervisor, {robot_module, opts}}

    # All hardware-facing subsystems share a restart budget under the
    # topology supervisor; its death triggers safety force-disarm.
    topology_supervisor_child = {BB.TopologySupervisor, {robot_module, opts}}

    [
      registry_child,
      pubsub_child,
      task_supervisor_child,
      command_supervisor_child,
      runtime_child,
      bridge_supervisor_child,
      topology_supervisor_child
    ]
  end
end
