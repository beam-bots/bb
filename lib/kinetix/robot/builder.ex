# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.Builder do
  @moduledoc """
  Builds an optimised `Kinetix.Robot` struct from DSL output.

  This module traverses the nested DSL structure and produces a flat,
  optimised representation suitable for kinematic computations.
  """

  alias Kinetix.Dsl
  alias Kinetix.Robot
  alias Kinetix.Robot.{Joint, Link, Topology, Units}

  @doc """
  Build a Robot struct from a robot module that uses the Kinetix DSL.
  """
  @spec build(module()) :: Robot.t()
  def build(robot_module) when is_atom(robot_module) do
    [root_dsl_link] = Dsl.Info.topology(robot_module)
    build_from_dsl(robot_module, root_dsl_link)
  end

  @doc """
  Build a Robot struct from a DSL root link.
  """
  @spec build_from_dsl(atom(), Dsl.Link.t()) :: Robot.t()
  def build_from_dsl(name, %Dsl.Link{} = root_dsl_link) do
    {links, joints, sensors, actuators} = collect_all(root_dsl_link)
    topology = build_topology(root_dsl_link.name, links, joints)

    %Robot{
      name: name,
      root_link: root_dsl_link.name,
      links: links,
      joints: joints,
      sensors: sensors,
      actuators: actuators,
      topology: topology
    }
  end

  defp collect_all(root_dsl_link) do
    acc = %{
      links: %{},
      joints: %{},
      sensors: %{},
      actuators: %{}
    }

    acc = collect_link(root_dsl_link, nil, acc)

    {acc.links, acc.joints, acc.sensors, acc.actuators}
  end

  defp collect_link(%Dsl.Link{} = dsl_link, parent_joint_name, acc) do
    link = convert_link(dsl_link, parent_joint_name)
    acc = put_in(acc.links[link.name], link)

    acc = collect_link_sensors(dsl_link.sensors, link.name, acc)

    Enum.reduce(dsl_link.joints, acc, fn dsl_joint, acc ->
      collect_joint(dsl_joint, link.name, acc)
    end)
  end

  defp collect_joint(%Dsl.Joint{} = dsl_joint, parent_link_name, acc) do
    child_link_name = dsl_joint.link.name
    joint = convert_joint(dsl_joint, parent_link_name, child_link_name)
    acc = put_in(acc.joints[joint.name], joint)

    acc = collect_joint_sensors(dsl_joint.sensors, joint.name, acc)
    acc = collect_actuators(dsl_joint.actuators, joint.name, acc)

    collect_link(dsl_joint.link, joint.name, acc)
  end

  defp collect_link_sensors(sensors, link_name, acc) do
    Enum.reduce(sensors, acc, fn %Dsl.Sensor{name: name}, acc ->
      put_in(acc.sensors[name], %{name: name, attached_to: {:link, link_name}})
    end)
  end

  defp collect_joint_sensors(sensors, joint_name, acc) do
    Enum.reduce(sensors, acc, fn %Dsl.Sensor{name: name}, acc ->
      put_in(acc.sensors[name], %{name: name, attached_to: {:joint, joint_name}})
    end)
  end

  defp collect_actuators(actuators, joint_name, acc) do
    Enum.reduce(actuators, acc, fn %Dsl.Actuator{name: name}, acc ->
      put_in(acc.actuators[name], %{name: name, joint: joint_name})
    end)
  end

  defp convert_link(%Dsl.Link{} = dsl_link, parent_joint_name) do
    %Link{
      name: dsl_link.name,
      parent_joint: parent_joint_name,
      child_joints: Enum.map(dsl_link.joints, & &1.name),
      mass: convert_mass(dsl_link.inertial),
      center_of_mass: convert_center_of_mass(dsl_link.inertial),
      inertia: convert_inertia(dsl_link.inertial),
      visual: convert_visual(dsl_link.visual),
      collisions: Enum.map(dsl_link.collisions, &convert_collision/1),
      sensors: Enum.map(dsl_link.sensors, & &1.name)
    }
  end

  defp convert_joint(%Dsl.Joint{} = dsl_joint, parent_link_name, child_link_name) do
    %Joint{
      name: dsl_joint.name,
      type: dsl_joint.type,
      parent_link: parent_link_name,
      child_link: child_link_name,
      origin: convert_origin(dsl_joint.origin),
      axis: convert_axis(dsl_joint.axis),
      limits: convert_limits(dsl_joint.limit, dsl_joint.type),
      dynamics: convert_dynamics(dsl_joint.dynamics, dsl_joint.type),
      sensors: Enum.map(dsl_joint.sensors, & &1.name),
      actuators: Enum.map(dsl_joint.actuators, & &1.name)
    }
  end

  defp convert_mass(nil), do: nil
  defp convert_mass(%Dsl.Inertial{mass: nil}), do: nil
  defp convert_mass(%Dsl.Inertial{mass: mass}), do: Units.to_kilograms(mass)

  defp convert_center_of_mass(nil), do: nil
  defp convert_center_of_mass(%Dsl.Inertial{origin: nil}), do: nil

  defp convert_center_of_mass(%Dsl.Inertial{origin: origin}) do
    {
      Units.to_meters(origin.x),
      Units.to_meters(origin.y),
      Units.to_meters(origin.z)
    }
  end

  defp convert_inertia(nil), do: nil
  defp convert_inertia(%Dsl.Inertial{inertia: nil}), do: nil

  defp convert_inertia(%Dsl.Inertial{inertia: inertia}) do
    %{
      ixx: Units.to_kilogram_square_meters(inertia.ixx),
      iyy: Units.to_kilogram_square_meters(inertia.iyy),
      izz: Units.to_kilogram_square_meters(inertia.izz),
      ixy: Units.to_kilogram_square_meters(inertia.ixy),
      ixz: Units.to_kilogram_square_meters(inertia.ixz),
      iyz: Units.to_kilogram_square_meters(inertia.iyz)
    }
  end

  defp convert_origin(nil), do: nil

  defp convert_origin(%Dsl.Origin{} = origin) do
    %{
      position: {
        Units.to_meters(origin.x),
        Units.to_meters(origin.y),
        Units.to_meters(origin.z)
      },
      orientation: {
        Units.to_radians(origin.roll),
        Units.to_radians(origin.pitch),
        Units.to_radians(origin.yaw)
      }
    }
  end

  defp convert_axis(nil), do: nil

  defp convert_axis(%Dsl.Axis{} = axis) do
    x = Units.to_meters(axis.x)
    y = Units.to_meters(axis.y)
    z = Units.to_meters(axis.z)

    magnitude = :math.sqrt(x * x + y * y + z * z)

    if magnitude > 0 do
      {x / magnitude, y / magnitude, z / magnitude}
    else
      {0.0, 0.0, 1.0}
    end
  end

  defp convert_limits(nil, _type), do: nil

  defp convert_limits(%Dsl.Limit{} = limit, type) when type in [:revolute, :continuous] do
    %{
      lower: Units.to_radians_or_nil(limit.lower),
      upper: Units.to_radians_or_nil(limit.upper),
      velocity: Units.to_radians_per_second(limit.velocity),
      effort: Units.to_newton_meters(limit.effort)
    }
  end

  defp convert_limits(%Dsl.Limit{} = limit, :prismatic) do
    %{
      lower: Units.to_meters_or_nil(limit.lower),
      upper: Units.to_meters_or_nil(limit.upper),
      velocity: Units.to_meters_per_second(limit.velocity),
      effort: Units.to_newton(limit.effort)
    }
  end

  defp convert_limits(%Dsl.Limit{} = limit, _type) do
    %{
      lower: nil,
      upper: nil,
      velocity: Units.to_radians_per_second(limit.velocity),
      effort: Units.to_newton_meters(limit.effort)
    }
  end

  defp convert_dynamics(nil, _type), do: nil

  defp convert_dynamics(%Dsl.Dynamics{} = dynamics, type)
       when type in [:revolute, :continuous] do
    %{
      damping: Units.to_rotational_damping_or_nil(dynamics.damping),
      friction: Units.to_newton_meters_or_nil(dynamics.friction)
    }
  end

  defp convert_dynamics(%Dsl.Dynamics{} = dynamics, type)
       when type in [:prismatic, :planar] do
    %{
      damping: Units.to_linear_damping_or_nil(dynamics.damping),
      friction: Units.to_newtons_or_nil(dynamics.friction)
    }
  end

  defp convert_dynamics(%Dsl.Dynamics{}, _type) do
    nil
  end

  defp convert_visual(nil), do: nil

  defp convert_visual(%Dsl.Visual{} = visual) do
    %{
      origin: convert_visual_origin(visual.origin),
      geometry: convert_geometry(visual.geometry),
      material: convert_material(visual.material)
    }
  end

  defp convert_visual_origin(nil), do: nil

  defp convert_visual_origin(%Dsl.Origin{} = origin) do
    position = {
      Units.to_meters(origin.x),
      Units.to_meters(origin.y),
      Units.to_meters(origin.z)
    }

    orientation = {
      Units.to_radians(origin.roll),
      Units.to_radians(origin.pitch),
      Units.to_radians(origin.yaw)
    }

    {position, orientation}
  end

  defp convert_collision(%Dsl.Collision{} = collision) do
    %{
      name: collision.name,
      origin: convert_visual_origin(collision.origin),
      geometry: convert_geometry(collision.geometry)
    }
  end

  defp convert_geometry(nil), do: nil

  defp convert_geometry(%Dsl.Box{} = box) do
    {:box,
     %{
       x: Units.to_meters(box.x),
       y: Units.to_meters(box.y),
       z: Units.to_meters(box.z)
     }}
  end

  defp convert_geometry(%Dsl.Cylinder{} = cylinder) do
    {:cylinder,
     %{
       radius: Units.to_meters(cylinder.radius),
       height: Units.to_meters(cylinder.height)
     }}
  end

  defp convert_geometry(%Dsl.Sphere{} = sphere) do
    {:sphere, %{radius: Units.to_meters(sphere.radius)}}
  end

  defp convert_geometry(%Dsl.Mesh{} = mesh) do
    {:mesh, %{filename: mesh.filename, scale: mesh.scale}}
  end

  defp convert_material(nil), do: nil

  defp convert_material(%Dsl.Material{} = material) do
    %{
      name: material.name,
      color: convert_color(material.color),
      texture: convert_texture(material.texture)
    }
  end

  defp convert_color(nil), do: nil

  defp convert_color(%Dsl.Color{} = color) do
    %{
      red: color.red,
      green: color.green,
      blue: color.blue,
      alpha: color.alpha
    }
  end

  defp convert_texture(nil), do: nil
  defp convert_texture(%Dsl.Texture{filename: filename}), do: filename

  defp build_topology(root_link_name, links, joints) do
    ctx = %{
      links: links,
      joints: joints,
      link_order: [],
      joint_order: [],
      paths: %{},
      depth: %{}
    }

    ctx = traverse_topology(root_link_name, [], 0, ctx)

    %Topology{
      link_order: Enum.reverse(ctx.link_order),
      joint_order: Enum.reverse(ctx.joint_order),
      paths: ctx.paths,
      depth: ctx.depth
    }
  end

  defp traverse_topology(link_name, current_path, current_depth, ctx) do
    link = Map.fetch!(ctx.links, link_name)
    link_path = current_path ++ [link_name]

    ctx = %{
      ctx
      | link_order: [link_name | ctx.link_order],
        paths: Map.put(ctx.paths, link_name, link_path),
        depth: Map.put(ctx.depth, link_name, current_depth)
    }

    Enum.reduce(link.child_joints, ctx, fn joint_name, ctx ->
      joint = Map.fetch!(ctx.joints, joint_name)
      joint_path = link_path ++ [joint_name]

      ctx = %{
        ctx
        | joint_order: [joint_name | ctx.joint_order],
          paths: Map.put(ctx.paths, joint_name, joint_path),
          depth: Map.put(ctx.depth, joint_name, current_depth + 1)
      }

      traverse_topology(joint.child_link, joint_path, current_depth + 1, ctx)
    end)
  end
end
