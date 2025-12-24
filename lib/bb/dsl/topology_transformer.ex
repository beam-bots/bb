# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.TopologyTransformer do
  @moduledoc """
  Validate and transform links as required.
  """
  use Spark.Dsl.Transformer
  alias BB.Cldr.Unit
  alias BB.Dsl.{Axis, Dynamics, Info, Joint, Limit, Link, ParamRef}
  alias Spark.{Dsl.Transformer, Error.DslError}

  @doc false
  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(BB.Dsl.SupervisorTransformer), do: true
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
        with {:ok, link, dsl, _path} <- recursive_transform(link, dsl, [:topology]) do
          {:ok, Transformer.replace_entity(dsl, [:topology], link)}
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

  defp recursive_transform(link, dsl, path) when is_struct(link, Link) do
    with {:ok, joints, dsl, _path} <-
           recursive_transform(link.joints, dsl, [link | [:link | path]]) do
      {:ok, %{link | joints: joints}, dsl, path}
    end
  end

  defp recursive_transform(joint, dsl, path)
       when is_struct(joint, Joint) and is_nil(joint.link) do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(dsl, :module),
       path: to_error_path([joint | [:joint | path]]),
       message: """
       All joints must connect to a child link
       """
     )}
  end

  defp recursive_transform(joint, dsl, path)
       when is_struct(joint, Joint) and is_nil(joint.limit) and
              joint.type in [:prismatic, :revolute] do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(dsl, :module),
       path: to_error_path([joint | [:joint | path]]),
       message: """
       Limits must be present for #{joint.type} joints
       """
     )}
  end

  defp recursive_transform(joint, dsl, path) when is_struct(joint, Joint) do
    inner_path = [joint | [:joint | path]]

    with {:ok, dynamics, dsl, _path} <- recursive_transform(joint.dynamics, dsl, inner_path),
         {:ok, axis, dsl, _path} <- recursive_transform(joint.axis, dsl, inner_path),
         {:ok, limit, dsl, _path} <- recursive_transform(joint.limit, dsl, inner_path),
         {:ok, link, dsl, _path} <- recursive_transform(joint.link, dsl, inner_path) do
      {:ok,
       %{
         joint
         | dynamics: dynamics,
           axis: axis,
           limit: limit,
           link: link
       }, dsl, path}
    end
  end

  defp recursive_transform(dynamics, dsl, [parent | _] = path)
       when is_struct(dynamics, Dynamics) and parent.type in [:prismatic, :planar] do
    inner_path = [:dynamics | path]

    with :ok <-
           validate_unit_or_nil(dynamics.damping, :newton_second_per_meter, dsl, [
             :damping | inner_path
           ]),
         :ok <- validate_unit_or_nil(dynamics.friction, :newton, dsl, [:friction | inner_path]) do
      {:ok, dynamics, dsl, path}
    end
  end

  defp recursive_transform(dynamics, dsl, [parent | _] = path)
       when is_struct(dynamics, Dynamics) and parent.type in [:revolute, :continuous] do
    inner_path = [:dynamics | path]

    with :ok <-
           validate_unit_or_nil(dynamics.damping, :newton_meter_second_per_degree, dsl, [
             :damping | inner_path
           ]),
         :ok <-
           validate_unit_or_nil(dynamics.friction, :newton_meter, dsl, [:friction | inner_path]) do
      {:ok, dynamics, dsl, path}
    end
  end

  defp recursive_transform(dynamics, dsl, [parent | _] = path)
       when is_struct(dynamics, Dynamics) do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(dsl, :module),
       path: to_error_path([:dynamics | path]),
       message: """
       Joint dynamics cannot be provided for #{parent.type} joints
       """
     )}
  end

  defp recursive_transform(axis, dsl, [parent | _] = path)
       when is_struct(axis, Axis) and parent.type == :fixed do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(dsl, :module),
       path: to_error_path([:axis | path]),
       module: """
       Cannot set an axis when parent joint is fixed.
       """
     )}
  end

  defp recursive_transform(axis, dsl, path) when is_struct(axis, Axis), do: {:ok, axis, dsl, path}

  defp recursive_transform(limit, dsl, [parent | _] = path)
       when is_struct(limit, Limit) and parent.type == :prismatic do
    inner_path = [:limit | path]

    with :ok <-
           validate_unit_or_nil(limit.lower, :meter, dsl, [:lower | inner_path]),
         :ok <-
           validate_unit_or_nil(limit.upper, :meter, dsl, [:upper | inner_path]),
         :ok <- validate_unit(limit.effort, :newton, dsl, [:effort | inner_path]),
         :ok <- validate_unit(limit.velocity, :meter_per_second, dsl, [:velocity | inner_path]) do
      {:ok, limit, dsl, path}
    end
  end

  defp recursive_transform(limit, dsl, [parent | _] = path)
       when is_struct(limit, Limit) and parent.type == :revolute do
    inner_path = [:limit | path]

    with :ok <- validate_unit_or_nil(limit.lower, :degree, dsl, [:lower | inner_path]),
         :ok <- validate_unit_or_nil(limit.upper, :degree, dsl, [:upper | inner_path]),
         :ok <- validate_unit(limit.effort, :newton_meter, dsl, [:effort | inner_path]),
         :ok <- validate_unit(limit.velocity, :radian_per_second, dsl, [:velocity | inner_path]) do
      {:ok, limit, dsl, path}
    end
  end

  defp recursive_transform(limit, dsl, [parent | _] = path)
       when is_struct(limit, Limit) and parent.type == :continuous do
    inner_path = [:limit | path]

    with :ok <-
           validate_nil(
             limit.lower,
             dsl,
             [:lower | inner_path],
             "Lower limit cannot be set when the parent joint is continuous"
           ),
         :ok <-
           validate_nil(
             limit.upper,
             dsl,
             [:upper | inner_path],
             "Lower limit cannot be set when the parent joint is continuous"
           ),
         :ok <- validate_unit(limit.effort, :newton_meter, dsl, [:effort | inner_path]),
         :ok <- validate_unit(limit.velocity, :radian_per_second, dsl, [:velocity | inner_path]) do
      {:ok, limit, dsl, path}
    end
  end

  defp recursive_transform(limit, dsl, [parent | _] = path)
       when is_struct(limit, Limit) and parent.type == :planar do
    inner_path = [:limit | path]

    with :ok <-
           validate_nil(
             limit.lower,
             dsl,
             [:lower | inner_path],
             "Lower limit cannot be set when the parent joint is planar"
           ),
         :ok <-
           validate_nil(
             limit.upper,
             dsl,
             [:upper | inner_path],
             "Lower limit cannot be set when the parent joint is planar"
           ),
         :ok <- validate_unit(limit.effort, :newton, dsl, [:effort | inner_path]),
         :ok <- validate_unit(limit.velocity, :meter_per_second, dsl, [:velocity | inner_path]) do
      {:ok, limit, dsl, path}
    end
  end

  defp recursive_transform(limit, dsl, [parent | _] = path)
       when is_struct(limit, Limit) and parent.type == :fixed do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(dsl, :module),
       path: to_error_path([:axis | path]),
       module: """
       Cannot set limits when parent joint is fixed.
       """
     )}
  end

  defp recursive_transform([], dsl, path), do: {:ok, [], dsl, path}

  defp recursive_transform(entities, dsl, path) when is_list(entities) do
    Enum.reduce_while(entities, {:ok, [], dsl, path}, fn entity, {:ok, entities, dsl, path} ->
      case recursive_transform(entity, dsl, path) do
        {:ok, entity, dsl, _path} -> {:cont, {:ok, [entity | entities], dsl, path}}
        {:error, reason} -> {:halt, {:error, reason}}
      end
    end)
  end

  defp recursive_transform(nil, dsl, path), do: {:ok, nil, dsl, path}

  defp validate_unit_or_nil(nil, _unit_name, _dsl, _path), do: :ok

  defp validate_unit_or_nil(unit, unit_name, dsl, path),
    do: validate_unit(unit, unit_name, dsl, path)

  # Skip validation for ParamRef - validated by ValidateParamRefs verifier
  defp validate_unit(%ParamRef{}, _unit_name, _dsl, _path), do: :ok

  defp validate_unit(unit, unit_name, dsl, path) do
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
         module: Transformer.get_persisted(dsl, :module),
         path: to_error_path(path),
         message: """
         Expected unit `#{Unit.to_string!(unit)}` to be compatible with #{readable_unit_name}
         """
       )}
    end
  end

  defp validate_nil(nil, _dsl, _path, _message), do: :ok

  defp validate_nil(_value, dsl, path, message) do
    {:error,
     DslError.exception(
       module: Transformer.get_persisted(dsl, :module),
       path: path,
       message: message
     )}
  end

  defp to_error_path(path) do
    path
    |> Enum.reverse()
    |> Enum.map(fn
      segment when is_atom(segment) -> segment
      %{name: name} -> name
    end)
  end
end
