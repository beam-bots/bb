# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.DefaultNameTransformer do
  @moduledoc "Ensures that the default robot name is present"
  use Spark.Dsl.Transformer

  alias BB.Dsl.{
    Actuator,
    Category,
    Collision,
    Command,
    Controller,
    Info,
    Joint,
    Link,
    Material,
    Sensor,
    State,
    Visual
  }

  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: true

  @doc false
  @impl true
  def transform(dsl) do
    with {:ok, dsl} <- maybe_set_settings_name(dsl),
         {:ok, dsl, counts} <- name_entities(dsl, [:sensors], %{}),
         {:ok, dsl, counts} <- name_entities(dsl, [:controllers], counts),
         {:ok, dsl, counts} <- name_entities(dsl, [:commands], counts),
         {:ok, dsl, counts} <- name_entities(dsl, [:states], counts),
         {:ok, dsl, _counts} <- name_entities(dsl, [:topology], counts) do
      {:ok, dsl}
    end
  end

  defp maybe_set_settings_name(dsl) do
    case Info.settings_name(dsl) do
      {:ok, _} ->
        {:ok, dsl}

      :error ->
        name =
          dsl
          |> Transformer.get_persisted(:module)
          |> short_name_for()

        {:ok, Transformer.set_option(dsl, [:settings], :name, name)}
    end
  end

  defp name_entities(dsl, path, counts) do
    dsl
    |> Transformer.get_entities(path)
    |> Enum.reduce_while({:ok, dsl, counts}, fn entity, {:ok, dsl, counts} ->
      case name_entity(entity, counts) do
        {:ok, entity, counts} ->
          {:cont, {:ok, Transformer.replace_entity(dsl, path, entity), counts}}

        {:error, reason} ->
          {:halt, {:error, reason}}
      end
    end)
  end

  defp name_entities(entities, counts) do
    Enum.reduce_while(entities, {:ok, [], counts}, fn entity, {:ok, entities, counts} ->
      case name_entity(entity, counts) do
        {:ok, entity, counts} -> {:cont, {:ok, [entity | entities], counts}}
        {:error, reason} -> {:halt, {:error, reason}}
      end
    end)
  end

  defp name_entity(link, counts) when is_struct(link, Link) do
    with {:ok, link, counts} <- maybe_set_name(link, :link, counts),
         {:ok, joints, counts} <- name_entities(link.joints, counts),
         {:ok, visual, counts} <- name_entity(link.visual, counts),
         {:ok, sensors, counts} <- name_entities(link.sensors, counts),
         {:ok, collisions, counts} <- name_entities(link.collisions, counts) do
      {:ok,
       %{
         link
         | joints: joints,
           visual: visual,
           sensors: sensors,
           collisions: collisions
       }, counts}
    end
  end

  defp name_entity(joint, counts) when is_struct(joint, Joint) do
    with {:ok, joint, counts} <- maybe_set_name(joint, :joint, counts),
         {:ok, link, counts} <- name_entity(joint.link, counts),
         {:ok, sensors, counts} <- name_entities(joint.sensors, counts),
         {:ok, actuators, counts} <- name_entities(joint.actuators, counts) do
      {:ok,
       %{
         joint
         | link: link,
           sensors: sensors,
           actuators: actuators
       }, counts}
    end
  end

  defp name_entity(sensor, counts) when is_struct(sensor, Sensor),
    do: maybe_set_name(sensor, :sensor, counts)

  defp name_entity(actuator, counts) when is_struct(actuator, Actuator),
    do: maybe_set_name(actuator, :actuator, counts)

  defp name_entity(visual, counts) when is_struct(visual, Visual) do
    with {:ok, material, counts} <- name_entity(visual.material, counts) do
      {:ok, %{visual | material: material}, counts}
    end
  end

  defp name_entity(material, counts) when is_struct(material, Material),
    do: maybe_set_name(material, :material, counts)

  defp name_entity(command, counts) when is_struct(command, Command),
    do: maybe_set_name(command, :command, counts)

  defp name_entity(controller, counts) when is_struct(controller, Controller),
    do: maybe_set_name(controller, :controller, counts)

  defp name_entity(collision, counts) when is_struct(collision, Collision),
    do: maybe_set_name(collision, :collision, counts)

  # Category and State entities always have explicit names, just pass through
  defp name_entity(category, counts) when is_struct(category, Category),
    do: {:ok, category, counts}

  defp name_entity(state, counts) when is_struct(state, State),
    do: {:ok, state, counts}

  defp name_entity(nil, counts), do: {:ok, nil, counts}

  defp maybe_set_name(entity, name, counts) when is_nil(entity.name) do
    counts = Map.update(counts, name, 0, &(&1 + 1))
    {:ok, %{entity | name: :"#{name}_#{counts[name]}"}, counts}
  end

  defp maybe_set_name(entity, _key, counts), do: {:ok, entity, counts}

  defp short_name_for(module) do
    module
    |> Module.split()
    |> List.last()
    |> Macro.underscore()
    |> String.to_atom()
  end
end
