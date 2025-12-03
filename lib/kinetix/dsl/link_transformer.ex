# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.LinkTransformer do
  @moduledoc """
  Validate and transform links as required.
  """
  use Spark.Dsl.Transformer
  alias Kinetix.Cldr.Unit
  alias Kinetix.Dsl.{Info, Joint, Link}
  alias Spark.{Dsl.Transformer, Error.DslError}

  @doc false
  @impl true
  def after?(Kinetix.Dsl.DefaultNameTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(Kinetix.Dsl.SupervisorTransformer), do: true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    dsl
    |> Info.topology()
    |> Enum.filter(&is_struct(&1, Link))
    |> case do
      [] ->
        {:ok, dsl}

      [link] ->
        state = %{dsl: dsl, link_count: 0, joint_count: 0, path: [:topology]}

        with {:ok, link, state} <- recursive_transform(link, state) do
          {:ok, Transformer.replace_entity(state.dsl, [:topology], link)}
        end

      links ->
        {:error,
         DslError.exception(
           module: Transformer.get_persisted(dsl, :module),
           path: [:topology],
           message: """
           There can only be one link at the root of the kinematic graph. You have supplied #{length(links)}
           """
         )}
    end
  end

  defp recursive_transform(link, state) when is_struct(link, Link) do
    with {:ok, link, state} <- maybe_set_link_name(link, state) do
      transform_link_joints(link, %{
        state
        | link_count: state.link_count + 1,
          path: [link.name, :link | state.path]
      })
    end
  end

  defp recursive_transform(joint, state) when is_struct(joint, Joint) do
    with {:ok, joint, state} <- maybe_set_joint_name(joint, state),
         {:ok, joint, new_state} <-
           transform_joint_dynamics(joint, %{state | path: [joint.name, :joint | state.path]}),
         {:ok, joint, new_state} <- transform_joint_limit(joint, new_state) do
      transform_joint_link(joint, %{
        new_state
        | joint_count: state.joint_count + 1,
          path: state.path
      })
    end
  end

  defp transform_joint_link(joint, state) when is_nil(joint.link) do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(state.dsl, :module),
       path: Enum.reverse(state.path),
       message: """
       All joints must connect to a child link
       """
     )}
  end

  defp transform_joint_link(joint, state) do
    with {:ok, link, state} <- recursive_transform(joint.link, state) do
      {:ok, %{joint | link: link}, state}
    end
  end

  defp transform_joint_dynamics(joint, state) when is_nil(joint.dynamics), do: {:ok, joint, state}

  defp transform_joint_dynamics(joint, state) when joint.type in [:prismatic, :planar] do
    with :ok <-
           validate_unit_or_nil(joint.dynamics.damping, :newton_second_per_meter, %{
             state
             | path: [:damping, :dynamics | state.path]
           }),
         :ok <-
           validate_unit_or_nil(joint.dynamics.friction, :newton, %{
             state
             | path: [:friction, :dynamics | state.path]
           }) do
      {:ok, joint, state}
    end
  end

  defp transform_joint_dynamics(joint, state) when joint.type in [:revolute, :continuous] do
    with :ok <-
           validate_unit_or_nil(joint.dynamics.damping, :newton_meter_second_per_degree, %{
             state
             | path: [:damping, :dynamics | state.path]
           }),
         :ok <-
           validate_unit_or_nil(joint.dynamics.friction, :newton_meter, %{
             state
             | path: [:friction, :dynamics | state.path]
           }) do
      {:ok, joint, state}
    end
  end

  defp transform_joint_dynamics(joint, state) do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(state.dsl, :module),
       path: Enum.reverse(state.path),
       message: """
       Joint dynamics cannot be provided for #{joint.type} joints
       """
     )}
  end

  defp validate_unit_or_nil(nil, _unit_name, _state), do: :ok
  defp validate_unit_or_nil(unit, unit_name, state), do: validate_unit(unit, unit_name, state)

  defp validate_unit(unit, unit_name, state) do
    if Unit.compatible?(unit, unit_name) do
      :ok
    else
      readable_unit_name =
        unit_name
        |> to_string()
        |> String.split("_")
        |> Enum.join(" ")

      {:error,
       DslError.exception(
         module: Transformer.get_persisted(state.dsl, :module),
         path: Enum.reverse(state.path),
         message: """
         Expected unit `#{Unit.to_string!(unit)}` to be compatible with #{readable_unit_name}
         """
       )}
    end
  end

  defp transform_joint_limit(joint, state)
       when is_nil(joint.limit) and joint.type in [:revolute, :prismatic] do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(state.dsl, :module),
       path: Enum.reverse([:limit | state.path]),
       message: """
       Limits must be present for #{joint.type} joints
       """
     )}
  end

  defp transform_joint_limit(joint, state) when is_nil(joint.limit), do: {:ok, joint, state}

  defp transform_joint_limit(joint, state) when joint.type == :revolute do
    with :ok <-
           validate_unit_or_nil(joint.limit.lower, :degree, %{
             state
             | path: [:lower, :limit | state.path]
           }),
         :ok <-
           validate_unit_or_nil(joint.limit.upper, :degree, %{
             state
             | path: [:upper, :limit | state.path]
           }) do
      {:ok, joint, state}
    end
  end

  defp transform_joint_limit(joint, state) when joint.type == :prismatic do
    with :ok <-
           validate_unit_or_nil(joint.limit.lower, :meter, %{
             state
             | path: [:lower, :limit | state.path]
           }),
         :ok <-
           validate_unit_or_nil(joint.limit.upper, :meter, %{
             state
             | path: [:upper, :limit | state.path]
           }) do
      {:ok, joint, state}
    end
  end

  defp transform_joint_limit(joint, state), do: {:ok, joint, state}

  defp maybe_set_joint_name(joint, state) when is_nil(joint.name) do
    {:ok, %{joint | name: :"joint_#{state.joint_count}"}, state}
  end

  defp maybe_set_joint_name(joint, state), do: {:ok, joint, state}

  defp maybe_set_link_name(link, state) when is_nil(link.name) do
    {:ok, %{link | name: :"link_#{state.link_count}"}, state}
  end

  defp maybe_set_link_name(link, state), do: {:ok, link, state}

  defp transform_link_joints(link, state) do
    Enum.reduce_while(link.joints, {:ok, %{link | joints: []}, state}, fn joint,
                                                                          {:ok, link, state} ->
      case recursive_transform(joint, state) do
        {:ok, joint, state} -> {:cont, {:ok, %{link | joints: [joint | link.joints]}, state}}
        {:error, reason} -> {:halt, {:error, reason}}
      end
    end)
  end
end
