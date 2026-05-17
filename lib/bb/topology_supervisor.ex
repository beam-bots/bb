# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.TopologySupervisor do
  @moduledoc """
  Supervisor for all hardware-facing subsystems of a robot.

  Groups the sensor, controller, and link supervisors so that the
  hardware-facing tree has its own restart budget. When the budget is
  exhausted this supervisor shuts down, signalling that hardware control
  is unrecoverable. `BB.Safety.Controller` monitors this supervisor and
  force-disarms the robot when it dies, leaving infrastructure processes
  (registry, pubsub, runtime, bridges) running so external systems can
  still observe the failure and call `BB.Safety.force_disarm/1`.

  The restart budget is configurable via the `topology_max_restarts` and
  `topology_max_seconds` settings on the robot DSL.
  """

  use Supervisor

  alias BB.Dsl.{Info, Link}
  alias BB.Safety.Controller, as: SafetyController

  def start_link({robot_module, opts}) do
    Supervisor.start_link(__MODULE__, {robot_module, opts},
      name: BB.Process.via(robot_module, __MODULE__)
    )
  end

  @impl true
  def init({robot_module, opts}) do
    settings = Info.settings(robot_module)

    :ok = SafetyController.register_topology_supervisor(robot_module)

    link_children =
      robot_module
      |> Info.topology()
      |> Enum.filter(&is_struct(&1, Link))
      |> Enum.map(fn link ->
        {BB.LinkSupervisor, {robot_module, link, [], opts}}
      end)

    children =
      [
        {BB.SensorSupervisor, {robot_module, opts}},
        {BB.ControllerSupervisor, {robot_module, opts}}
      ] ++ link_children

    Supervisor.init(children,
      strategy: :one_for_one,
      max_restarts: settings.topology_max_restarts,
      max_seconds: settings.topology_max_seconds
    )
  end
end
