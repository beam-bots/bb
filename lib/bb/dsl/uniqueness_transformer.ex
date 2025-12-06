# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.UniquenessTransformer do
  @moduledoc """
  Validates that all entity names are globally unique across the robot.

  This includes links, joints, sensors, actuators, and controllers - all entities
  that get registered in the process registry. Commands are not included since
  they're not registered processes.
  """
  use Spark.Dsl.Transformer
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @doc false
  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(BB.Dsl.RobotTransformer), do: true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    names =
      [[:sensors], [:controllers], [:topology]]
      |> Enum.reduce(%{}, fn path, names ->
        dsl
        |> Transformer.get_entities(path)
        |> retrieve_names(path, names)
      end)

    dupes =
      names
      |> Enum.reject(fn
        {_name, []} -> true
        {_name, [_]} -> true
        {_name, _paths} -> false
      end)

    if Enum.empty?(dupes) do
      {:ok, dsl}
    else
      dupes =
        dupes
        |> Enum.map(fn {name, paths} ->
          paths =
            paths
            |> Enum.reverse()
            |> Enum.map_join("\n", &"   - `#{inspect(&1)}`")

          " - `#{inspect(name)}`:\n#{paths}\n"
        end)

      {:error,
       DslError.exception(
         module: Transformer.get_persisted(dsl, :module),
         message: """
         Entities with duplicate names found at the following paths:

         #{dupes}
         """
       )}
    end
  end

  defp retrieve_names(entity, path, names) when is_map_key(entity, :name) do
    names =
      names
      |> Map.update(entity.name, [path], &[path | &1])

    path = [entity.name | path]

    entity
    |> Map.drop([:name, :__struct__])
    |> Enum.reduce(names, fn {key, value}, names ->
      retrieve_names(value, [key | path], names)
    end)
  end

  defp retrieve_names(entity, path, names) when is_map(entity) do
    entity
    |> Map.delete(:__struct__)
    |> Enum.reduce(names, fn {key, value}, names ->
      retrieve_names(value, [key | path], names)
    end)
  end

  defp retrieve_names(entities, path, names) when is_list(entities) do
    entities
    |> Enum.reduce(names, fn value, names ->
      retrieve_names(value, path, names)
    end)
  end

  defp retrieve_names(_value, _path, names), do: names
end
