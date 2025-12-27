# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.BridgeSupervisor do
  @moduledoc """
  Supervisor for parameter protocol bridges.

  Groups all bridges defined in the `parameters` section under a single
  supervisor for fault isolation. A flapping bridge (e.g., due to network
  issues) won't exhaust the root supervisor's restart budget.
  """

  use Supervisor

  alias BB.Dsl.{Bridge, Info}

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
      |> Info.parameters()
      |> Enum.filter(&is_struct(&1, Bridge))
      |> Enum.flat_map(fn bridge ->
        build_bridge_child(robot_module, bridge, simulation_mode)
      end)

    Supervisor.init(children, strategy: :one_for_one)
  end

  defp build_bridge_child(robot_module, bridge, nil = _simulation_mode) do
    [BB.Process.bridge_child_spec(robot_module, bridge.name, bridge.child_spec, [])]
  end

  defp build_bridge_child(robot_module, bridge, _simulation_mode) do
    case bridge.simulation do
      :omit -> []
      :mock -> [BB.Process.bridge_child_spec(robot_module, bridge.name, BB.Sim.Bridge, [])]
      :start -> [BB.Process.bridge_child_spec(robot_module, bridge.name, bridge.child_spec, [])]
    end
  end
end
