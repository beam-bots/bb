# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ValidateLimitUnitsTransformer do
  @moduledoc """
  Validates that the units provided in a joint's `limit` block are compatible
  with the joint's type.

  The DSL schema permits both angular and linear units in `limit` fields so
  that the same entity definition serves rotational and translational joints.
  This transformer rejects the wrong-axis combinations before the
  `BB.Dsl.RobotTransformer` attempts unit conversion:

  - Rotational joints (`:revolute`, `:continuous`) require angular units
    (`degree`, `degree_per_second`, `newton_meter`, `degree_per_square_second`).
  - Linear joints (`:prismatic`) require linear units (`meter`,
    `meter_per_second`, `newton`, `meter_per_square_second`).

  Joints without degrees of freedom (`:fixed`, `:floating`, `:planar`) are not
  checked — they typically have no `limit` block at all.
  """

  use Spark.Dsl.Transformer

  alias BB.Dsl.{Joint, Limit, Link, ParamRef, Transmission}
  alias BB.Unit
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @rotational_types [:revolute, :continuous]
  @linear_types [:prismatic]

  @rotational_units %{
    lower: :degree,
    upper: :degree,
    velocity: :degree_per_second,
    effort: :newton_meter,
    acceleration: :degree_per_square_second
  }

  @linear_units %{
    lower: :meter,
    upper: :meter,
    velocity: :meter_per_second,
    effort: :newton,
    acceleration: :meter_per_square_second
  }

  @doc false
  @impl true
  def after?(BB.Dsl.TopologyTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(BB.Dsl.RobotTransformer), do: true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    dsl
    |> Transformer.get_entities([:topology])
    |> walk_joints([])
    |> Enum.reduce_while({:ok, dsl}, fn {joint, path}, {:ok, dsl} ->
      case validate_joint(joint, path, module) do
        :ok -> {:cont, {:ok, dsl}}
        {:error, _} = err -> {:halt, err}
      end
    end)
  end

  defp walk_joints(entities, path_prefix) do
    Enum.flat_map(entities, fn
      %Link{} = link ->
        link_path = path_prefix ++ [:link, link.name]
        Enum.flat_map(link.joints, &walk_joints([&1], link_path))

      %Joint{} = joint ->
        joint_path = path_prefix ++ [:joint, joint.name]
        nested = if joint.link, do: walk_joints([joint.link], path_prefix), else: []
        [{joint, joint_path} | nested]

      _ ->
        []
    end)
  end

  defp validate_joint(%Joint{type: type} = joint, path, module)
       when type in @rotational_types do
    with :ok <- check_limit(joint.limit, @rotational_units, path, module, type) do
      check_attachment_transmissions(joint, :degree, path, module, type)
    end
  end

  defp validate_joint(%Joint{type: type} = joint, path, module)
       when type in @linear_types do
    with :ok <- check_limit(joint.limit, @linear_units, path, module, type) do
      check_attachment_transmissions(joint, :meter, path, module, type)
    end
  end

  defp validate_joint(%Joint{}, _path, _module), do: :ok

  defp check_limit(nil, _expected_units, _path, _module, _joint_type), do: :ok

  defp check_limit(%Limit{} = limit, expected_units, path, module, joint_type) do
    check_fields(limit, expected_units, path, module, joint_type)
  end

  defp check_attachment_transmissions(%Joint{} = joint, expected_unit, path, module, joint_type) do
    attachments =
      Enum.map(joint.actuators, &{:actuator, &1}) ++ Enum.map(joint.sensors, &{:sensor, &1})

    Enum.reduce_while(attachments, :ok, fn {kind, attachment}, :ok ->
      attachment_path = path ++ [kind, attachment.name]

      case check_transmission_offset(
             attachment.transmission,
             expected_unit,
             attachment_path,
             module,
             joint_type
           ) do
        :ok -> {:cont, :ok}
        {:error, _} = err -> {:halt, err}
      end
    end)
  end

  defp check_transmission_offset(nil, _expected_unit, _path, _module, _joint_type), do: :ok

  defp check_transmission_offset(
         %Transmission{offset: offset},
         expected_unit,
         path,
         module,
         joint_type
       ) do
    case check_field(offset, :transmission, :offset, expected_unit, path, module, joint_type) do
      {:cont, :ok} -> :ok
      {:halt, error} -> error
    end
  end

  defp check_fields(limit, expected_units, path, module, joint_type) do
    Enum.reduce_while(expected_units, :ok, fn {field, expected_unit}, :ok ->
      check_field(Map.get(limit, field), :limit, field, expected_unit, path, module, joint_type)
    end)
  end

  defp check_field(nil, _section, _field, _expected_unit, _path, _module, _joint_type),
    do: {:cont, :ok}

  defp check_field(%ParamRef{}, _section, _field, _expected_unit, _path, _module, _joint_type),
    do: {:cont, :ok}

  defp check_field(
         %Localize.Unit{} = value,
         section,
         field,
         expected_unit,
         path,
         module,
         joint_type
       ) do
    if Unit.compatible?(value, expected_unit) do
      {:cont, :ok}
    else
      {:halt,
       {:error, mismatch_error(module, path, section, field, value, expected_unit, joint_type)}}
    end
  end

  defp check_field(_value, _section, _field, _expected_unit, _path, _module, _joint_type),
    do: {:cont, :ok}

  defp mismatch_error(module, path, section, field, value, expected_unit, joint_type) do
    DslError.exception(
      module: module,
      path: path ++ [section, field],
      message: """
      The unit `#{value.name}` provided for `#{field}` is not compatible with a `#{joint_type}` joint.

      Expected a unit compatible with `#{expected_unit}`.
      """
    )
  end
end
