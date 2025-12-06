# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ParameterTransformer do
  @moduledoc """
  Generates parameter schema and default values from DSL definitions.

  This transformer processes the `parameters` section and component entities,
  generating:
  - `__bb_parameter_schema__/0` - Returns the Spark.Options schema for validation
  - `__bb_default_parameters__/0` - Returns default values as `{path, value}` tuples

  Parameters are collected from:
  - The `parameters` section (groups and params)
  - Controllers in the `controllers` section
  - Sensors in the `sensors` section
  - Sensors and actuators within the topology

  At runtime, these are used by `BB.Robot.Runtime` to register parameters with
  proper validation schemas.
  """
  use Spark.Dsl.Transformer
  alias BB.Dsl.{Joint, Link, Param, ParamGroup}
  alias BB.Unit
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(BB.Dsl.RobotTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    # Collect from parameters section
    {param_schema, param_defaults} =
      dsl
      |> Transformer.get_entities([:parameters])
      |> collect_parameters([])

    # Collect from controllers section
    {ctrl_schema, ctrl_defaults} =
      dsl
      |> Transformer.get_entities([:controllers])
      |> collect_component_params([:controller])

    # Collect from sensors section (robot-level)
    {sensor_schema, sensor_defaults} =
      dsl
      |> Transformer.get_entities([:sensors])
      |> collect_component_params([:sensor])

    # Collect from topology (links, joints, sensors, actuators)
    {topo_schema, topo_defaults} =
      dsl
      |> Transformer.get_entities([:topology])
      |> collect_topology_params([])

    schema_opts = param_schema ++ ctrl_schema ++ sensor_schema ++ topo_schema
    defaults = param_defaults ++ ctrl_defaults ++ sensor_defaults ++ topo_defaults

    if Enum.empty?(schema_opts) do
      inject_empty_functions(dsl)
    else
      inject_parameter_functions(dsl, schema_opts, defaults)
    end
  end

  defp collect_parameters(entities, path_prefix) do
    Enum.reduce(entities, {[], []}, fn entity, {schema_acc, defaults_acc} ->
      case entity do
        %Param{} = param ->
          {param_schema, param_defaults} = process_param(param, path_prefix)
          {schema_acc ++ param_schema, defaults_acc ++ param_defaults}

        %ParamGroup{} = group ->
          group_path = path_prefix ++ [group.name]

          {nested_schema, nested_defaults} =
            collect_parameters(group.params ++ group.groups, group_path)

          {schema_acc ++ nested_schema, defaults_acc ++ nested_defaults}

        _ ->
          {schema_acc, defaults_acc}
      end
    end)
  end

  defp collect_component_params(components, prefix) do
    Enum.reduce(components, {[], []}, fn component, {schema_acc, defaults_acc} ->
      path = prefix ++ [component.name]
      {comp_schema, comp_defaults} = collect_params_from_entity(component.params, path)
      {schema_acc ++ comp_schema, defaults_acc ++ comp_defaults}
    end)
  end

  defp collect_topology_params(entities, path_prefix) do
    Enum.reduce(entities, {[], []}, fn entity, acc ->
      collect_from_topology_entity(entity, path_prefix, acc)
    end)
  end

  defp collect_from_topology_entity(%Link{} = link, path_prefix, {schema_acc, defaults_acc}) do
    link_path = path_prefix ++ [:link, link.name]

    {sensor_schema, sensor_defaults} =
      collect_component_params(link.sensors, link_path ++ [:sensor])

    {joint_schema, joint_defaults} = collect_topology_params(link.joints, link_path)

    {schema_acc ++ sensor_schema ++ joint_schema,
     defaults_acc ++ sensor_defaults ++ joint_defaults}
  end

  defp collect_from_topology_entity(%Joint{} = joint, path_prefix, {schema_acc, defaults_acc}) do
    joint_path = path_prefix ++ [:joint, joint.name]

    {sensor_schema, sensor_defaults} =
      collect_component_params(Map.get(joint, :sensors, []), joint_path ++ [:sensor])

    {actuator_schema, actuator_defaults} =
      collect_component_params(Map.get(joint, :actuators, []), joint_path ++ [:actuator])

    {nested_schema, nested_defaults} = collect_from_nested_link(joint, path_prefix)

    {schema_acc ++ sensor_schema ++ actuator_schema ++ nested_schema,
     defaults_acc ++ sensor_defaults ++ actuator_defaults ++ nested_defaults}
  end

  defp collect_from_topology_entity(_entity, _path_prefix, acc), do: acc

  defp collect_from_nested_link(joint, path_prefix) do
    case Map.get(joint, :link) do
      nil -> {[], []}
      nested_link -> collect_topology_params([nested_link], path_prefix)
    end
  end

  defp collect_params_from_entity(params, path_prefix) when is_list(params) do
    Enum.reduce(params, {[], []}, fn param, {schema_acc, defaults_acc} ->
      {param_schema, param_defaults} = process_param(param, path_prefix)
      {schema_acc ++ param_schema, defaults_acc ++ param_defaults}
    end)
  end

  defp collect_params_from_entity(nil, _path_prefix), do: {[], []}

  defp process_param(%Param{} = param, path_prefix) do
    path = path_prefix ++ [param.name]
    schema_opts = build_schema_opts(param)

    defaults =
      if param.default != nil do
        [{path, param.default}]
      else
        []
      end

    {[{path, schema_opts}], defaults}
  end

  defp build_schema_opts(%Param{} = param) do
    opts = [type: convert_param_type(param.type)]

    opts = if param.doc, do: Keyword.put(opts, :doc, param.doc), else: opts
    opts = if param.default != nil, do: Keyword.put(opts, :default, param.default), else: opts

    opts
  end

  defp convert_param_type({:unit, unit_type}) do
    Unit.Option.unit_type(compatible: unit_type)
  end

  defp convert_param_type(type), do: type

  defp inject_empty_functions(dsl) do
    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @doc false
         @spec __bb_parameter_schema__() :: [{[atom()], keyword()}]
         def __bb_parameter_schema__, do: []

         @doc false
         @spec __bb_default_parameters__() :: [{[atom()], term()}]
         def __bb_default_parameters__, do: []
       end
     )}
  end

  defp inject_parameter_functions(dsl, schema_opts, defaults) do
    schema_data = Macro.escape(schema_opts)
    defaults_data = Macro.escape(defaults)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @bb_parameter_schema unquote(schema_data)
         @bb_default_parameters unquote(defaults_data)

         @doc """
         Returns the parameter schema for validation.

         The schema is a list of `{path, opts}` tuples where `opts` is a keyword
         list compatible with `Spark.Options`.
         """
         @spec __bb_parameter_schema__() :: [{[atom()], keyword()}]
         def __bb_parameter_schema__, do: @bb_parameter_schema

         @doc """
         Returns default parameter values.

         Returns a list of `{path, value}` tuples for parameters that have defaults.
         """
         @spec __bb_default_parameters__() :: [{[atom()], term()}]
         def __bb_default_parameters__, do: @bb_default_parameters
       end
     )}
  end
end
