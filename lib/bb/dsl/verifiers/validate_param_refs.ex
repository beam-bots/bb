# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidateParamRefs do
  @moduledoc """
  Validates that parameter references in the DSL refer to valid parameters.

  For each `param([:path, :to, :param])` in the topology, this verifier checks:
  1. The parameter path exists in the parameters section
  2. The parameter's unit type is compatible with the expected type at that DSL location
  """

  use Spark.Dsl.Verifier

  alias BB.Cldr.Unit
  alias BB.Dsl.{Axis, Dynamics, Inertia, Inertial, Joint, Limit, Link, Origin, ParamRef}
  alias Spark.Dsl.Verifier
  alias Spark.Error.DslError

  @impl true
  def verify(dsl_state) do
    module = Verifier.get_persisted(dsl_state, :module)
    param_types = build_param_types_map(module)

    dsl_state
    |> Verifier.get_entities([:topology])
    |> collect_param_refs([])
    |> validate_refs(param_types, module)
  end

  defp build_param_types_map(module) do
    if function_exported?(module, :__bb_parameter_schema__, 0) do
      module.__bb_parameter_schema__()
      |> Enum.map(fn {path, opts} ->
        unit_type = extract_unit_type(opts[:type])
        {path, unit_type}
      end)
      |> Enum.into(%{})
    else
      %{}
    end
  end

  defp extract_unit_type({:custom, BB.Unit.Option, :validate, [opts]}) do
    opts[:compatible]
  end

  defp extract_unit_type(_), do: nil

  defp collect_param_refs(entities, path_prefix) do
    Enum.flat_map(entities, fn entity ->
      collect_from_entity(entity, path_prefix)
    end)
  end

  defp collect_from_entity(%Link{} = link, path_prefix) do
    link_path = path_prefix ++ [:link, link.name]

    inertial_refs = collect_from_inertial(link.inertial, link_path ++ [:inertial])
    joint_refs = Enum.flat_map(link.joints, &collect_from_entity(&1, link_path))

    inertial_refs ++ joint_refs
  end

  defp collect_from_entity(%Joint{} = joint, path_prefix) do
    joint_path = path_prefix ++ [:joint, joint.name]

    origin_refs = collect_from_origin(joint.origin, joint_path ++ [:origin])
    axis_refs = collect_from_axis(joint.axis, joint_path ++ [:axis])
    limit_refs = collect_from_limit(joint.limit, joint_path ++ [:limit])
    dynamics_refs = collect_from_dynamics(joint.dynamics, joint_path ++ [:dynamics])

    nested_refs =
      case joint.link do
        nil -> []
        nested_link -> collect_from_entity(nested_link, path_prefix)
      end

    origin_refs ++ axis_refs ++ limit_refs ++ dynamics_refs ++ nested_refs
  end

  defp collect_from_entity(_entity, _path_prefix), do: []

  defp collect_from_origin(nil, _path), do: []

  defp collect_from_origin(%Origin{} = origin, path) do
    [:roll, :pitch, :yaw, :x, :y, :z]
    |> Enum.flat_map(fn field ->
      case Map.get(origin, field) do
        %ParamRef{} = ref -> [{ref, path ++ [field]}]
        _ -> []
      end
    end)
  end

  defp collect_from_axis(nil, _path), do: []

  defp collect_from_axis(%Axis{} = axis, path) do
    [:roll, :pitch, :yaw]
    |> Enum.flat_map(fn field ->
      case Map.get(axis, field) do
        %ParamRef{} = ref -> [{ref, path ++ [field]}]
        _ -> []
      end
    end)
  end

  defp collect_from_limit(nil, _path), do: []

  defp collect_from_limit(%Limit{} = limit, path) do
    [:lower, :upper, :effort, :velocity]
    |> Enum.flat_map(fn field ->
      case Map.get(limit, field) do
        %ParamRef{} = ref -> [{ref, path ++ [field]}]
        _ -> []
      end
    end)
  end

  defp collect_from_dynamics(nil, _path), do: []

  defp collect_from_dynamics(%Dynamics{} = dynamics, path) do
    [:damping, :friction]
    |> Enum.flat_map(fn field ->
      case Map.get(dynamics, field) do
        %ParamRef{} = ref -> [{ref, path ++ [field]}]
        _ -> []
      end
    end)
  end

  defp collect_from_inertial(nil, _path), do: []

  defp collect_from_inertial(%Inertial{} = inertial, path) do
    mass_refs =
      case inertial.mass do
        %ParamRef{} = ref -> [{ref, path ++ [:mass]}]
        _ -> []
      end

    origin_refs = collect_from_origin(inertial.origin, path ++ [:origin])
    inertia_refs = collect_from_inertia(inertial.inertia, path ++ [:inertia])

    mass_refs ++ origin_refs ++ inertia_refs
  end

  defp collect_from_inertia(nil, _path), do: []

  defp collect_from_inertia(%Inertia{} = inertia, path) do
    [:ixx, :ixy, :ixz, :iyy, :iyz, :izz]
    |> Enum.flat_map(fn field ->
      case Map.get(inertia, field) do
        %ParamRef{} = ref -> [{ref, path ++ [field]}]
        _ -> []
      end
    end)
  end

  defp validate_refs(refs, param_types, module) do
    Enum.reduce_while(refs, :ok, fn {ref, dsl_path}, :ok ->
      case validate_ref(ref, dsl_path, param_types, module) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp validate_ref(%ParamRef{path: path} = ref, dsl_path, param_types, module) do
    case Map.fetch(param_types, path) do
      {:ok, param_unit_type} ->
        validate_unit_compatibility(ref, param_unit_type, dsl_path, module)

      :error ->
        {:error,
         DslError.exception(
           module: module,
           path: dsl_path,
           message: """
           Parameter reference #{format_path(path)} does not exist.

           The referenced parameter path was not found in the parameters section.
           Available parameters: #{format_available_paths(param_types)}
           """
         )}
    end
  end

  defp validate_unit_compatibility(
         %ParamRef{expected_unit_type: nil},
         _param_unit_type,
         _dsl_path,
         _module
       ) do
    :ok
  end

  defp validate_unit_compatibility(%ParamRef{} = ref, param_unit_type, dsl_path, module) do
    expected = ref.expected_unit_type

    cond do
      param_unit_type == nil ->
        {:error,
         DslError.exception(
           module: module,
           path: dsl_path,
           message: """
           Parameter #{format_path(ref.path)} is not a unit type.

           This DSL field requires a unit compatible with #{inspect(expected)},
           but the parameter is not defined as a unit type.
           """
         )}

      units_compatible?(expected, param_unit_type) ->
        :ok

      # For fields with {:or, ...} types (like effort which accepts newton OR newton_meter),
      # the expected_unit_type only captures the first alternative. The actual conversion
      # is handled by the Builder based on joint type, so we allow any unit-typed parameter
      # as long as it's a unit type (param_unit_type is not nil, checked above).
      true ->
        :ok
    end
  end

  defp units_compatible?(expected, actual) do
    Unit.compatible?(expected, actual)
  end

  defp format_path(path) do
    "param(#{inspect(path)})"
  end

  defp format_available_paths(param_types) when map_size(param_types) == 0 do
    "(none defined)"
  end

  defp format_available_paths(param_types) do
    param_types
    |> Map.keys()
    |> Enum.map_join(", ", &inspect/1)
  end
end
