# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Urdf.Exporter do
  @moduledoc """
  Export a BB robot definition to URDF XML format.
  """

  alias BB.Robot
  alias BB.Robot.{Joint, Link}
  alias BB.Urdf.Xml

  @doc """
  Export a robot module to URDF XML string.

  The module must use `BB` and have a `robot/0` function.
  """
  @spec export(module()) :: {:ok, String.t()} | {:error, term()}
  def export(robot_module) when is_atom(robot_module) do
    with {:ok, _} <- ensure_compiled(robot_module),
         {:ok, robot} <- get_robot(robot_module) do
      export_robot(robot)
    end
  end

  @doc """
  Export a Robot struct to URDF XML string.
  """
  @spec export_robot(Robot.t()) :: {:ok, String.t()}
  def export_robot(%Robot{} = robot) do
    links = Robot.links_in_order(robot) |> Enum.map(&build_link_element/1)
    joints = Robot.joints_in_order(robot) |> Enum.map(&build_joint_element/1)

    xml =
      Xml.element(:robot, [name: format_robot_name(robot.name)], links ++ joints)
      |> Xml.to_string()

    {:ok, xml}
  end

  defp format_robot_name(name) when is_atom(name) do
    name
    |> Atom.to_string()
    |> String.replace_prefix("Elixir.", "")
  end

  defp ensure_compiled(module) do
    case Code.ensure_compiled(module) do
      {:module, _} -> {:ok, module}
      {:error, reason} -> {:error, {:module_not_found, module, reason}}
    end
  end

  defp get_robot(module) do
    if function_exported?(module, :robot, 0) do
      {:ok, module.robot()}
    else
      {:error, {:not_a_bb_module, module}}
    end
  end

  defp build_link_element(%Link{} = link) do
    children = [
      build_inertial_element(link),
      build_visual_element(link.visual),
      Enum.map(link.collisions, &build_collision_element/1)
    ]

    Xml.element(:link, [name: Atom.to_string(link.name)], children)
  end

  defp build_inertial_element(%Link{mass: nil, inertia: nil}), do: nil

  defp build_inertial_element(%Link{mass: mass, center_of_mass: com, inertia: inertia}) do
    children = [
      if(com, do: build_origin_element({com, {0.0, 0.0, 0.0}})),
      if(mass, do: Xml.element(:mass, value: Xml.format_float(mass))),
      build_inertia_element(inertia)
    ]

    Xml.element(:inertial, [], children)
  end

  defp build_inertia_element(nil), do: nil

  defp build_inertia_element(inertia) do
    Xml.element(:inertia,
      ixx: Xml.format_float(inertia.ixx),
      iyy: Xml.format_float(inertia.iyy),
      izz: Xml.format_float(inertia.izz),
      ixy: Xml.format_float(inertia.ixy),
      ixz: Xml.format_float(inertia.ixz),
      iyz: Xml.format_float(inertia.iyz)
    )
  end

  defp build_visual_element(nil), do: nil

  defp build_visual_element(visual) do
    children = [
      build_origin_element(visual.origin),
      build_geometry_element(visual.geometry),
      build_material_element(visual.material)
    ]

    Xml.element(:visual, [], children)
  end

  defp build_collision_element(collision) do
    attrs = if collision.name, do: [name: Atom.to_string(collision.name)], else: []

    children = [
      build_origin_element(collision.origin),
      build_geometry_element(collision.geometry)
    ]

    Xml.element(:collision, attrs, children)
  end

  defp build_geometry_element(nil), do: nil

  defp build_geometry_element({:box, %{x: x, y: y, z: z}}) do
    Xml.element(:geometry, [], [
      Xml.element(:box, size: Xml.format_xyz({x, y, z}))
    ])
  end

  defp build_geometry_element({:cylinder, %{radius: r, height: h}}) do
    Xml.element(:geometry, [], [
      Xml.element(:cylinder, radius: Xml.format_float(r), length: Xml.format_float(h))
    ])
  end

  defp build_geometry_element({:sphere, %{radius: r}}) do
    Xml.element(:geometry, [], [
      Xml.element(:sphere, radius: Xml.format_float(r))
    ])
  end

  # Capsules are exported as cylinders (URDF doesn't have native capsule support)
  # Total height = length + 2 * radius
  defp build_geometry_element({:capsule, %{radius: r, length: l}}) do
    total_height = l + 2 * r

    Xml.element(:geometry, [], [
      Xml.element(:cylinder, radius: Xml.format_float(r), length: Xml.format_float(total_height))
    ])
  end

  defp build_geometry_element({:mesh, %{filename: filename, scale: scale}}) do
    scale_str = Xml.format_xyz({scale, scale, scale})

    Xml.element(:geometry, [], [
      Xml.element(:mesh, filename: filename, scale: scale_str)
    ])
  end

  defp build_material_element(nil), do: nil

  defp build_material_element(material) do
    children = [
      build_color_element(material.color),
      build_texture_element(material.texture)
    ]

    Xml.element(:material, [name: Atom.to_string(material.name)], children)
  end

  defp build_color_element(nil), do: nil

  defp build_color_element(%{red: r, green: g, blue: b, alpha: a}) do
    rgba =
      "#{Xml.format_float(r)} #{Xml.format_float(g)} #{Xml.format_float(b)} #{Xml.format_float(a)}"

    Xml.element(:color, rgba: rgba)
  end

  defp build_texture_element(nil), do: nil
  defp build_texture_element(filename), do: Xml.element(:texture, filename: filename)

  defp build_joint_element(%Joint{} = joint) do
    children = [
      build_origin_element(joint.origin),
      Xml.element(:parent, link: Atom.to_string(joint.parent_link)),
      Xml.element(:child, link: Atom.to_string(joint.child_link)),
      build_axis_element(joint),
      build_limit_element(joint),
      build_dynamics_element(joint)
    ]

    Xml.element(
      :joint,
      [name: Atom.to_string(joint.name), type: Atom.to_string(joint.type)],
      children
    )
  end

  defp build_origin_element(nil), do: nil

  defp build_origin_element(%{position: pos, orientation: orient}) do
    Xml.element(:origin, xyz: Xml.format_xyz(pos), rpy: Xml.format_xyz(orient))
  end

  defp build_origin_element({pos, orient}) do
    Xml.element(:origin, xyz: Xml.format_xyz(pos), rpy: Xml.format_xyz(orient))
  end

  defp build_axis_element(%Joint{type: :fixed}), do: nil
  defp build_axis_element(%Joint{axis: nil}), do: nil
  defp build_axis_element(%Joint{axis: axis}), do: Xml.element(:axis, xyz: Xml.format_xyz(axis))

  defp build_limit_element(%Joint{type: :fixed}), do: nil
  defp build_limit_element(%Joint{limits: nil}), do: nil

  defp build_limit_element(%Joint{type: :continuous, limits: limits}) do
    Xml.element(:limit,
      effort: Xml.format_float(limits.effort),
      velocity: Xml.format_float(limits.velocity)
    )
  end

  defp build_limit_element(%Joint{limits: limits}) do
    attrs =
      [
        lower: limits.lower && Xml.format_float(limits.lower),
        upper: limits.upper && Xml.format_float(limits.upper),
        effort: Xml.format_float(limits.effort),
        velocity: Xml.format_float(limits.velocity)
      ]
      |> Enum.reject(fn {_k, v} -> is_nil(v) end)

    Xml.element(:limit, attrs)
  end

  defp build_dynamics_element(%Joint{type: :fixed}), do: nil
  defp build_dynamics_element(%Joint{dynamics: nil}), do: nil

  defp build_dynamics_element(%Joint{dynamics: dynamics}) do
    attrs =
      [
        damping: dynamics.damping && Xml.format_float(dynamics.damping),
        friction: dynamics.friction && Xml.format_float(dynamics.friction)
      ]
      |> Enum.reject(fn {_k, v} -> is_nil(v) end)

    if Enum.empty?(attrs), do: nil, else: Xml.element(:dynamics, attrs)
  end
end
