defmodule Kinetix.Dsl.RobotTransformer do
  @moduledoc """
  Builds and persists the optimised Robot struct at compile-time.

  This transformer runs after the LinkTransformer to ensure the DSL is fully
  validated, then builds the optimised `Kinetix.Robot` struct and injects
  an accessor function into the robot module.

  ## Validation

  This transformer validates that all entity names are globally unique across
  the robot. This includes links, joints, sensors, and actuators.
  """
  use Spark.Dsl.Transformer
  alias Kinetix.Dsl.{Joint, Link, Sensor}
  alias Kinetix.Robot.Builder
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @doc false
  @impl true
  def after?(Kinetix.Dsl.LinkTransformer), do: true
  def after?(Kinetix.Dsl.SupervisorTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    dsl
    |> Transformer.get_entities([:robot])
    |> Enum.filter(&is_struct(&1, Link))
    |> case do
      [] ->
        {:ok, dsl}

      [root_link] ->
        with :ok <- validate_unique_names(root_link, dsl, module) do
          robot = Builder.build_from_dsl(module, root_link)
          inject_robot_accessor(dsl, module, robot)
        end
    end
  end

  defp validate_unique_names(root_link, dsl, module) do
    names = collect_all_names(root_link, dsl)

    names
    |> Enum.frequencies()
    |> Enum.filter(fn {_name, count} -> count > 1 end)
    |> case do
      [] ->
        :ok

      duplicates ->
        duplicate_names = Enum.map(duplicates, fn {name, _} -> name end)

        {:error,
         DslError.exception(
           module: module,
           path: [:robot],
           message: """
           All entity names must be unique across the robot.

           The following names are used more than once: #{inspect(duplicate_names)}

           This includes links, joints, sensors, and actuators.
           """
         )}
    end
  end

  defp collect_all_names(root_link, dsl) do
    robot_sensors =
      dsl
      |> Transformer.get_entities([:robot])
      |> Enum.filter(&is_struct(&1, Sensor))
      |> Enum.map(& &1.name)

    link_names = collect_names_from_link(root_link)

    robot_sensors ++ link_names
  end

  defp collect_names_from_link(%Link{} = link) do
    link_sensors = Enum.map(link.sensors, & &1.name)

    joint_names =
      Enum.flat_map(link.joints, fn joint ->
        collect_names_from_joint(joint)
      end)

    [link.name | link_sensors] ++ joint_names
  end

  defp collect_names_from_joint(%Joint{} = joint) do
    joint_sensors = Enum.map(joint.sensors, & &1.name)
    joint_actuators = Enum.map(joint.actuators, & &1.name)
    child_link_names = collect_names_from_link(joint.link)

    [joint.name | joint_sensors] ++ joint_actuators ++ child_link_names
  end

  defp inject_robot_accessor(dsl, module, robot) do
    robot_data = Macro.escape(robot)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @kinetix_robot unquote(robot_data)

         @doc """
         Returns the optimised robot representation.

         This struct is built at compile-time from the DSL definition and contains:
         - All physical values converted to SI base units (floats)
         - Flat maps for O(1) lookup of links, joints, sensors, and actuators
         - Pre-computed topology metadata for efficient traversal

         ## Examples

             robot = #{unquote(module)}.robot()
             link = Kinetix.Robot.get_link(robot, :base_link)
             joint = Kinetix.Robot.get_joint(robot, :shoulder)

         """
         @spec robot() :: Kinetix.Robot.t()
         def robot, do: @kinetix_robot
       end
     )}
  end
end
