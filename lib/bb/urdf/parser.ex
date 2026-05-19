# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Urdf.Parser do
  @moduledoc """
  Parse a URDF XML document into a plain-map intermediate representation.

  The result is intentionally close to the URDF wire format: link/joint
  lists are flat (joints reference parent and child link names), values are
  floats in SI base units, and unsupported URDF features are collected as
  warnings rather than raised as errors. The `BB.Urdf.Importer` consumes this
  representation and produces a BB DSL module.
  """

  require Record

  Record.defrecordp(
    :xml_element,
    :xmlElement,
    Record.extract(:xmlElement, from_lib: "xmerl/include/xmerl.hrl")
  )

  Record.defrecordp(
    :xml_attribute,
    :xmlAttribute,
    Record.extract(:xmlAttribute, from_lib: "xmerl/include/xmerl.hrl")
  )

  @type origin :: %{xyz: {float, float, float}, rpy: {float, float, float}}
  @type geometry ::
          {:box, %{size: {float, float, float}}}
          | {:cylinder, %{radius: float, length: float}}
          | {:sphere, %{radius: float}}
          | {:mesh, %{filename: String.t(), scale: float}}

  @type robot :: %{
          name: String.t(),
          links: [link],
          joints: [joint],
          transmissions: %{optional(String.t()) => transmission},
          materials: %{optional(String.t()) => material},
          warnings: [String.t()]
        }
  @type transmission :: %{
          name: String.t() | nil,
          joint: String.t(),
          reduction: float
        }
  @type link :: %{
          name: String.t(),
          visual: visual | nil,
          collisions: [collision],
          inertial: inertial | nil
        }
  @type visual :: %{
          name: String.t() | nil,
          origin: origin | nil,
          geometry: geometry | nil,
          material: material | nil
        }
  @type collision :: %{
          name: String.t() | nil,
          origin: origin | nil,
          geometry: geometry | nil
        }
  @type inertial :: %{
          origin: origin | nil,
          mass: float | nil,
          inertia: %{
            ixx: float,
            iyy: float,
            izz: float,
            ixy: float,
            ixz: float,
            iyz: float
          }
        }
  @type joint :: %{
          name: String.t(),
          type: atom,
          parent: String.t(),
          child: String.t(),
          origin: origin | nil,
          axis: {float, float, float} | nil,
          limit:
            %{
              lower: float | nil,
              upper: float | nil,
              effort: float | nil,
              velocity: float | nil
            }
            | nil,
          dynamics: %{damping: float | nil, friction: float | nil} | nil,
          mimic: %{joint: String.t(), multiplier: float, offset: float} | nil
        }
  @type material :: %{
          name: String.t() | nil,
          color: %{red: float, green: float, blue: float, alpha: float} | nil,
          texture: String.t() | nil
        }

  @doc """
  Parse a URDF XML file from disk.
  """
  @spec parse_file(Path.t()) :: {:ok, robot} | {:error, term}
  def parse_file(path) do
    with {:ok, content} <- File.read(path) do
      parse_string(content)
    end
  end

  @doc """
  Parse a URDF XML string.
  """
  @spec parse_string(String.t()) :: {:ok, robot} | {:error, term}
  def parse_string(xml) when is_binary(xml) do
    {doc, _rest} = :xmerl_scan.string(String.to_charlist(xml), space: :normalize)
    {:ok, parse_robot(doc)}
  catch
    :exit, reason -> {:error, {:xml_parse_error, reason}}
  end

  defp parse_robot(xml_element(name: :robot) = element) do
    name = attr(element, :name, "robot")
    children = children(element)
    materials = parse_top_level_materials(children)

    {links, link_warnings} =
      children
      |> filter_by_name(:link)
      |> Enum.map_reduce([], fn link_el, acc ->
        {link, warnings} = parse_link(link_el, materials)
        {link, warnings ++ acc}
      end)

    {joints, joint_warnings} =
      children
      |> filter_by_name(:joint)
      |> Enum.map_reduce([], fn joint_el, acc ->
        {joint, warnings} = parse_joint(joint_el)
        {joint, warnings ++ acc}
      end)

    {transmissions, transmission_warnings} =
      children
      |> filter_by_name(:transmission)
      |> Enum.map_reduce([], fn el, acc ->
        {parsed, warnings} = parse_transmission(el)
        {parsed, warnings ++ acc}
      end)

    transmissions_by_joint =
      transmissions
      |> Enum.reject(&is_nil/1)
      |> Enum.into(%{}, fn t -> {t.joint, t} end)

    gazebo_warnings =
      children
      |> Enum.flat_map(fn
        xml_element(name: :gazebo) -> ["skipping <gazebo> extension block"]
        _ -> []
      end)

    %{
      name: to_string(name),
      links: links,
      joints: joints,
      transmissions: transmissions_by_joint,
      materials: materials,
      warnings:
        Enum.reverse(link_warnings) ++
          Enum.reverse(joint_warnings) ++
          Enum.reverse(transmission_warnings) ++
          gazebo_warnings
    }
  end

  defp parse_transmission(xml_element(name: :transmission) = element) do
    name = attr(element, :name, nil)
    children = children(element)

    type =
      case first_by_name(children, :type) do
        nil -> nil
        el -> el |> text_content() |> String.trim()
      end

    joint_name =
      case first_by_name(children, :joint) do
        nil -> nil
        el -> el |> attr(:name) |> to_string()
      end

    actuators = filter_by_name(children, :actuator)

    cond do
      joint_name == nil ->
        {nil, ["skipping <transmission> #{inspect(to_string(name || ""))}: missing <joint>"]}

      type != nil and not simple_transmission?(type) ->
        {nil,
         [
           "skipping <transmission> #{inspect(to_string(name || ""))}: type #{inspect(type)} is not supported (only SimpleTransmission)"
         ]}

      length(actuators) > 1 ->
        {nil,
         [
           "skipping <transmission> #{inspect(to_string(name || ""))}: coupled transmissions (multiple <actuator>) are not supported"
         ]}

      true ->
        reduction =
          case actuators do
            [] -> 1.0
            [actuator | _] -> extract_reduction(actuator)
          end

        {%{
           name: name && to_string(name),
           joint: joint_name,
           reduction: reduction
         }, []}
    end
  end

  defp simple_transmission?(type) do
    String.ends_with?(type, "SimpleTransmission")
  end

  defp extract_reduction(actuator_element) do
    case actuator_element |> children() |> first_by_name(:mechanicalReduction) do
      nil -> 1.0
      el -> el |> text_content() |> String.trim() |> parse_float()
    end
  end

  defp text_content(xml_element(content: content)) do
    content
    |> Enum.map(fn
      {:xmlText, _, _, _, value, _} -> to_string(value)
      _ -> ""
    end)
    |> Enum.join("")
  end

  defp parse_top_level_materials(children) do
    children
    |> filter_by_name(:material)
    |> Enum.reduce(%{}, fn material_el, acc ->
      material = parse_material(material_el)

      case material.name do
        nil -> acc
        name -> Map.put(acc, name, material)
      end
    end)
  end

  defp parse_link(xml_element(name: :link) = element, materials) do
    name = attr(element, :name)
    children = children(element)

    {visual, visual_warnings} =
      case first_by_name(children, :visual) do
        nil -> {nil, []}
        el -> parse_visual(el, materials)
      end

    collisions = children |> filter_by_name(:collision) |> Enum.map(&parse_collision/1)

    inertial =
      case first_by_name(children, :inertial) do
        nil -> nil
        el -> parse_inertial(el)
      end

    {%{
       name: to_string(name),
       visual: visual,
       collisions: collisions,
       inertial: inertial
     }, visual_warnings}
  end

  defp parse_visual(xml_element(name: :visual) = element, materials) do
    name = attr(element, :name, nil)
    children = children(element)
    origin = first_by_name(children, :origin) |> parse_origin()
    geometry = first_by_name(children, :geometry) |> parse_geometry()

    {material, warnings} =
      case first_by_name(children, :material) do
        nil ->
          {nil, []}

        material_el ->
          parsed = parse_material(material_el)
          resolve_material(parsed, materials)
      end

    {%{
       name: name && to_string(name),
       origin: origin,
       geometry: geometry,
       material: material
     }, warnings}
  end

  defp resolve_material(%{color: nil, texture: nil, name: name} = parsed, materials)
       when is_binary(name) do
    case Map.get(materials, name) do
      nil ->
        {parsed, ["material #{inspect(name)} referenced by name but not defined at top level"]}

      defined ->
        {Map.merge(parsed, Map.take(defined, [:color, :texture])), []}
    end
  end

  defp resolve_material(parsed, _materials), do: {parsed, []}

  defp parse_collision(xml_element(name: :collision) = element) do
    name = attr(element, :name, nil)
    children = children(element)

    %{
      name: name && to_string(name),
      origin: first_by_name(children, :origin) |> parse_origin(),
      geometry: first_by_name(children, :geometry) |> parse_geometry()
    }
  end

  defp parse_inertial(xml_element(name: :inertial) = element) do
    children = children(element)

    mass =
      case first_by_name(children, :mass) do
        nil -> nil
        el -> el |> attr(:value, "0") |> parse_float()
      end

    inertia =
      case first_by_name(children, :inertia) do
        nil ->
          nil

        el ->
          %{
            ixx: el |> attr(:ixx, "0") |> parse_float(),
            iyy: el |> attr(:iyy, "0") |> parse_float(),
            izz: el |> attr(:izz, "0") |> parse_float(),
            ixy: el |> attr(:ixy, "0") |> parse_float(),
            ixz: el |> attr(:ixz, "0") |> parse_float(),
            iyz: el |> attr(:iyz, "0") |> parse_float()
          }
      end

    %{
      origin: first_by_name(children, :origin) |> parse_origin(),
      mass: mass,
      inertia: inertia
    }
  end

  defp parse_material(xml_element(name: :material) = element) do
    name = attr(element, :name, nil)
    children = children(element)

    color =
      case first_by_name(children, :color) do
        nil ->
          nil

        el ->
          [r, g, b, a] = el |> attr(:rgba, "0 0 0 1") |> parse_floats(4)
          %{red: r, green: g, blue: b, alpha: a}
      end

    texture =
      case first_by_name(children, :texture) do
        nil -> nil
        el -> el |> attr(:filename, nil) |> to_string_or_nil()
      end

    %{
      name: name && to_string(name),
      color: color,
      texture: texture
    }
  end

  defp parse_joint(xml_element(name: :joint) = element) do
    name = attr(element, :name)
    type = element |> attr(:type, "fixed") |> to_string() |> String.to_atom()
    children = children(element)

    parent =
      case first_by_name(children, :parent) do
        nil -> nil
        el -> el |> attr(:link) |> to_string()
      end

    child =
      case first_by_name(children, :child) do
        nil -> nil
        el -> el |> attr(:link) |> to_string()
      end

    origin = first_by_name(children, :origin) |> parse_origin()
    axis = parse_axis(first_by_name(children, :axis))
    limit = parse_limit(first_by_name(children, :limit))
    dynamics = parse_dynamics(first_by_name(children, :dynamics))
    mimic = parse_mimic(first_by_name(children, :mimic))

    safety_warnings =
      case first_by_name(children, :safety_controller) do
        nil -> []
        _ -> ["joint #{inspect(to_string(name))}: <safety_controller> is not supported"]
      end

    {%{
       name: to_string(name),
       type: type,
       parent: parent,
       child: child,
       origin: origin,
       axis: axis,
       limit: limit,
       dynamics: dynamics,
       mimic: mimic
     }, safety_warnings}
  end

  defp parse_origin(nil), do: nil

  defp parse_origin(xml_element() = element) do
    xyz = element |> attr(:xyz, "0 0 0") |> parse_floats(3) |> List.to_tuple()
    rpy = element |> attr(:rpy, "0 0 0") |> parse_floats(3) |> List.to_tuple()
    %{xyz: xyz, rpy: rpy}
  end

  defp parse_axis(nil), do: nil

  defp parse_axis(xml_element() = element) do
    element |> attr(:xyz, "1 0 0") |> parse_floats(3) |> List.to_tuple()
  end

  defp parse_limit(nil), do: nil

  defp parse_limit(xml_element() = element) do
    %{
      lower: element |> attr(:lower, nil) |> maybe_float(),
      upper: element |> attr(:upper, nil) |> maybe_float(),
      effort: element |> attr(:effort, nil) |> maybe_float(),
      velocity: element |> attr(:velocity, nil) |> maybe_float()
    }
  end

  defp parse_dynamics(nil), do: nil

  defp parse_dynamics(xml_element() = element) do
    %{
      damping: element |> attr(:damping, nil) |> maybe_float(),
      friction: element |> attr(:friction, nil) |> maybe_float()
    }
  end

  defp parse_mimic(nil), do: nil

  defp parse_mimic(xml_element() = element) do
    %{
      joint: element |> attr(:joint) |> to_string(),
      multiplier: element |> attr(:multiplier, "1") |> parse_float(),
      offset: element |> attr(:offset, "0") |> parse_float()
    }
  end

  defp parse_geometry(nil), do: nil

  defp parse_geometry(xml_element() = element) do
    case children(element) do
      [xml_element(name: :box) = el | _] ->
        size = el |> attr(:size, "0 0 0") |> parse_floats(3) |> List.to_tuple()
        {:box, %{size: size}}

      [xml_element(name: :cylinder) = el | _] ->
        {:cylinder,
         %{
           radius: el |> attr(:radius, "0") |> parse_float(),
           length: el |> attr(:length, "0") |> parse_float()
         }}

      [xml_element(name: :sphere) = el | _] ->
        {:sphere, %{radius: el |> attr(:radius, "0") |> parse_float()}}

      [xml_element(name: :mesh) = el | _] ->
        {:mesh,
         %{
           filename: el |> attr(:filename) |> to_string(),
           scale: el |> attr(:scale, "1 1 1") |> parse_scale()
         }}

      _ ->
        nil
    end
  end

  defp parse_scale(value) do
    case parse_floats(value, 3) do
      [s, _, _] -> s
      _ -> parse_float(value)
    end
  end

  defp children(xml_element(content: content)) do
    Enum.filter(content, fn
      xml_element() -> true
      _ -> false
    end)
  end

  defp filter_by_name(elements, name) do
    Enum.filter(elements, fn xml_element(name: n) -> n == name end)
  end

  defp first_by_name(elements, name) do
    Enum.find(elements, fn xml_element(name: n) -> n == name end)
  end

  defp attr(xml_element(attributes: attrs), key, default \\ nil) do
    attr_value(attrs, key, default)
  end

  defp attr_value(attrs, key, default) do
    Enum.find_value(attrs, default, fn
      xml_attribute(name: ^key, value: value) -> value
      _ -> nil
    end)
  end

  defp parse_float(value) when is_list(value), do: value |> List.to_string() |> parse_float()

  defp parse_float(value) when is_binary(value) do
    case Float.parse(value) do
      {f, _} -> f
      :error -> 0.0
    end
  end

  defp maybe_float(nil), do: nil
  defp maybe_float(value), do: parse_float(value)

  defp parse_floats(value, count) when is_list(value),
    do: value |> List.to_string() |> parse_floats(count)

  defp parse_floats(value, count) when is_binary(value) do
    value
    |> String.split(~r/\s+/, trim: true)
    |> Enum.map(&parse_float/1)
    |> pad_or_trim(count)
  end

  defp pad_or_trim(list, count) do
    list = Enum.take(list, count)
    list ++ List.duplicate(0.0, max(0, count - length(list)))
  end

  defp to_string_or_nil(nil), do: nil
  defp to_string_or_nil(value), do: to_string(value)
end
