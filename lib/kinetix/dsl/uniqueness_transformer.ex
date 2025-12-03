# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.UniquenessTransformer do
  @moduledoc """
  Validates that all entity names are globally unique across the robot.

  This includes links, joints, sensors, actuators, and controllers - all entities
  that get registered in the process registry. Commands are not included since
  they're not registered processes.
  """
  use Spark.Dsl.Transformer
  alias Kinetix.Dsl.{Controller, Info, Joint, Link, Sensor}
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @doc false
  @impl true
  def after?(Kinetix.Dsl.LinkTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(Kinetix.Dsl.RobotTransformer), do: true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    dsl
    |> Info.topology()
    |> Enum.filter(&is_struct(&1, Link))
    |> case do
      [] ->
        {:ok, dsl}

      [root_link] ->
        with :ok <- validate_unique_names(root_link, dsl, module) do
          {:ok, dsl}
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
           path: [:topology],
           message: """
           All entity names must be unique across the robot.

           The following names are used more than once: #{inspect(duplicate_names)}

           This includes links, joints, sensors, actuators, and controllers.
           """
         )}
    end
  end

  defp collect_all_names(root_link, dsl) do
    # Robot-level sensors from the robot sensors section
    robot_sensors =
      dsl
      |> Info.sensors()
      |> Enum.filter(&is_struct(&1, Sensor))
      |> Enum.map(& &1.name)

    # Robot-level controllers from the controllers section
    robot_controllers =
      dsl
      |> Info.controllers()
      |> Enum.filter(&is_struct(&1, Controller))
      |> Enum.map(& &1.name)

    # Names from the link hierarchy
    link_names = collect_names_from_link(root_link)

    robot_sensors ++ robot_controllers ++ link_names
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
end
