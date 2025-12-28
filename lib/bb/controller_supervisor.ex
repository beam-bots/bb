# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.ControllerSupervisor do
  @moduledoc """
  Supervisor for robot-level controllers.

  Groups all controllers defined in the `controllers` section under a single
  supervisor for fault isolation. A flapping controller won't exhaust
  the root supervisor's restart budget.
  """

  use Supervisor

  alias BB.Dsl.Info

  def start_link({robot_module, opts}) do
    Supervisor.start_link(__MODULE__, {robot_module, opts},
      name: BB.Process.via(robot_module, __MODULE__)
    )
  end

  @impl true
  def init({robot_module, opts}) do
    simulation_mode = Keyword.get(opts, :simulation)

    children =
      robot_module
      |> Info.controllers()
      |> Enum.flat_map(fn controller ->
        build_controller_child(robot_module, controller, simulation_mode, opts)
      end)

    Supervisor.init(children, strategy: :one_for_one)
  end

  defp build_controller_child(robot_module, controller, nil = _simulation_mode, opts) do
    [
      BB.Process.child_spec(
        robot_module,
        controller.name,
        controller.child_spec,
        [],
        :controller,
        opts
      )
    ]
  end

  defp build_controller_child(robot_module, controller, _simulation_mode, opts) do
    case controller.simulation do
      :omit ->
        []

      :mock ->
        [
          BB.Process.child_spec(
            robot_module,
            controller.name,
            BB.Sim.Controller,
            [],
            :controller,
            opts
          )
        ]

      :start ->
        [
          BB.Process.child_spec(
            robot_module,
            controller.name,
            controller.child_spec,
            [],
            :controller,
            opts
          )
        ]
    end
  end
end
