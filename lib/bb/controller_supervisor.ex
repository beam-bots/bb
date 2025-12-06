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
  def init({robot_module, _opts}) do
    children =
      robot_module
      |> Info.controllers()
      |> Enum.map(fn controller ->
        BB.Process.child_spec(robot_module, controller.name, controller.child_spec, [])
      end)

    Supervisor.init(children, strategy: :one_for_one)
  end
end
