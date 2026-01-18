# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Info do
  @moduledoc false
  use Spark.InfoGenerator,
    extension: BB.Dsl,
    sections: [
      :robot,
      :topology,
      :settings,
      :sensors,
      :controllers,
      :commands,
      :parameters,
      :states
    ]

  alias Spark.Dsl.Extension

  @doc """
  Returns the settings for the robot module.
  """
  @spec settings(module) :: %{
          registry_module: module,
          registry_options: keyword,
          supervisor_module: module,
          parameter_store: module | {module, keyword} | nil,
          auto_disarm_on_error: boolean
        }
  def settings(robot_module) do
    registry_options =
      Extension.get_opt(robot_module, [:settings], :registry_options) ||
        [partitions: System.schedulers_online()]

    %{
      registry_module: Extension.get_opt(robot_module, [:settings], :registry_module, Registry),
      registry_options: registry_options,
      supervisor_module:
        Extension.get_opt(robot_module, [:settings], :supervisor_module, Supervisor),
      parameter_store: Extension.get_opt(robot_module, [:settings], :parameter_store),
      auto_disarm_on_error:
        Extension.get_opt(robot_module, [:settings], :auto_disarm_on_error, true)
    }
  end

  @doc """
  Returns the list of defined states for the robot module.

  Includes the built-in `:idle` state plus any custom states defined
  in the `states` section.
  """
  @spec states(module) :: [BB.Dsl.State.t()]
  def states(robot_module) do
    robot_module.__bb_states__()
  end

  @doc """
  Returns the list of defined state names for the robot module.
  """
  @spec state_names(module) :: [atom]
  def state_names(robot_module) do
    robot_module.__bb_state_names__()
  end

  @doc """
  Returns the initial operational state for the robot module.
  """
  @spec initial_state(module) :: atom
  def initial_state(robot_module) do
    robot_module.__bb_initial_state__()
  end

  @doc """
  Returns the list of defined command categories for the robot module.

  Includes the built-in `:default` category plus any custom categories
  defined in the `commands` section.
  """
  @spec categories(module) :: [BB.Dsl.Category.t()]
  def categories(robot_module) do
    robot_module.__bb_categories__()
  end

  @doc """
  Returns a map of category names to their concurrency limits.
  """
  @spec category_limits(module) :: %{atom => pos_integer}
  def category_limits(robot_module) do
    robot_module.__bb_category_limits__()
  end
end
