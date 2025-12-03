defmodule Kinetix.Dsl.Info do
  @moduledoc false
  use Spark.InfoGenerator, extension: Kinetix.Dsl, sections: [:robot]

  alias Spark.Dsl.Extension

  @doc """
  Returns the settings for the robot module.
  """
  @spec settings(module) :: %{
          registry_module: module,
          registry_options: keyword,
          supervisor_module: module
        }
  def settings(robot_module) do
    registry_options =
      Extension.get_opt(robot_module, [:robot, :settings], :registry_options) ||
        [partitions: System.schedulers_online()]

    %{
      registry_module:
        Extension.get_opt(robot_module, [:robot, :settings], :registry_module, Registry),
      registry_options: registry_options,
      supervisor_module:
        Extension.get_opt(robot_module, [:robot, :settings], :supervisor_module, Supervisor)
    }
  end
end
