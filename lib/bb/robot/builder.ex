# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Builder do
  @moduledoc """
  Builds an optimised `BB.Robot` struct from DSL output.

  This module traverses the nested DSL structure and produces a flat,
  optimised representation suitable for kinematic computations.
  """

  alias BB.Dsl
  alias BB.Dsl.ParamRef
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.{Joint, Link, Topology, Units}

  @doc """
  Build a Robot struct from a robot module that uses the BB DSL.
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
    {links, joints, sensors, actuators, param_subscriptions} = collect_all(root_dsl_link)
    topology = build_topology(root_dsl_link.name, links, joints)

    %Robot{
      name: name,
      root_link: root_dsl_link.name,
      links: links,
      joints: joints,
      sensors: sensors,
      actuators: actuators,
      topology: topology,
      param_subscriptions: param_subscriptions
    }
  end

  defp collect_all(root_dsl_link) do
    acc = %{
      links: %{},
      joints: %{},
      sensors: %{},
      actuators: %{},
      param_subscriptions: %{}
    }

    acc = collect_link(root_dsl_link, nil, acc)

    {acc.links, acc.joints, acc.sensors, acc.actuators, acc.param_subscriptions}
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
    {joint, param_subs} = convert_joint(dsl_joint, parent_link_name, child_link_name)
    acc = put_in(acc.joints[joint.name], joint)
    acc = merge_param_subscriptions(acc, param_subs)

    acc = collect_joint_sensors(dsl_joint.sensors, joint.name, acc)
    acc = collect_actuators(dsl_joint.actuators, joint.name, acc)

    collect_link(dsl_joint.link, joint.name, acc)
  end

  defp merge_param_subscriptions(acc, new_subs) do
    merged =
      Enum.reduce(new_subs, acc.param_subscriptions, fn {path, location}, subs ->
        Map.update(subs, path, [location], &[location | &1])
      end)

    %{acc | param_subscriptions: merged}
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
    joint_name = dsl_joint.name

    {origin, origin_subs} = convert_origin(dsl_joint.origin, joint_name)
    {axis, axis_subs} = convert_axis(dsl_joint.axis, joint_name)
    {limits, limits_subs} = convert_limits(dsl_joint.limit, dsl_joint.type, joint_name)
    {dynamics, dynamics_subs} = convert_dynamics(dsl_joint.dynamics, dsl_joint.type, joint_name)

    joint = %Joint{
      name: joint_name,
      type: dsl_joint.type,
      parent_link: parent_link_name,
      child_link: child_link_name,
      origin: origin,
      axis: axis,
      limits: limits,
      dynamics: dynamics,
      sensors: Enum.map(dsl_joint.sensors, & &1.name),
      actuators: Enum.map(dsl_joint.actuators, & &1.name)
    }

    param_subs = origin_subs ++ axis_subs ++ limits_subs ++ dynamics_subs
    {joint, param_subs}
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

  defp convert_origin(nil, _joint_name), do: {nil, []}

  defp convert_origin(%Dsl.Origin{} = origin, joint_name) do
    {x, x_subs} = convert_value_with_ref(origin.x, &Units.to_meters/1, joint_name, [:origin, :x])
    {y, y_subs} = convert_value_with_ref(origin.y, &Units.to_meters/1, joint_name, [:origin, :y])
    {z, z_subs} = convert_value_with_ref(origin.z, &Units.to_meters/1, joint_name, [:origin, :z])

    {roll, roll_subs} =
      convert_value_with_ref(origin.roll, &Units.to_radians/1, joint_name, [:origin, :roll])

    {pitch, pitch_subs} =
      convert_value_with_ref(origin.pitch, &Units.to_radians/1, joint_name, [:origin, :pitch])

    {yaw, yaw_subs} =
      convert_value_with_ref(origin.yaw, &Units.to_radians/1, joint_name, [:origin, :yaw])

    converted = %{
      position: {x, y, z},
      orientation: {roll, pitch, yaw}
    }

    subs = x_subs ++ y_subs ++ z_subs ++ roll_subs ++ pitch_subs ++ yaw_subs
    {converted, subs}
  end

  defp convert_value_with_ref(%ParamRef{path: path}, _converter, joint_name, field_path) do
    {nil, [{path, {:joint, joint_name, field_path}}]}
  end

  defp convert_value_with_ref(value, converter, _joint_name, _field_path) do
    {converter.(value), []}
  end

  defp convert_axis(nil, _joint_name), do: {nil, []}

  defp convert_axis(%Dsl.Axis{} = axis, joint_name) do
    # Check if any values are ParamRefs - axis computation needs all values
    has_param_ref =
      Enum.any?([axis.roll, axis.pitch, axis.yaw], &is_struct(&1, ParamRef))

    if has_param_ref do
      # Collect subscriptions for param refs, return nil for axis (resolved at runtime)
      subs = collect_axis_param_refs(axis, joint_name)
      {nil, subs}
    else
      roll = Units.to_radians(axis.roll)
      pitch = Units.to_radians(axis.pitch)
      yaw = Units.to_radians(axis.yaw)

      # Build rotation matrix from Euler angles and apply to default Z axis
      rotation =
        Transform.rotation_x(roll)
        |> Transform.compose(Transform.rotation_y(pitch))
        |> Transform.compose(Transform.rotation_z(yaw))

      axis_vec3 = Transform.apply_to_point(rotation, Vec3.unit_z())
      axis_tuple = {Vec3.x(axis_vec3), Vec3.y(axis_vec3), Vec3.z(axis_vec3)}
      {axis_tuple, []}
    end
  end

  defp collect_axis_param_refs(axis, joint_name) do
    [:roll, :pitch, :yaw]
    |> Enum.flat_map(fn field ->
      case Map.get(axis, field) do
        %ParamRef{path: path} -> [{path, {:joint, joint_name, [:axis, field]}}]
        _ -> []
      end
    end)
  end

  defp convert_limits(nil, _type, _joint_name), do: {nil, []}

  defp convert_limits(%Dsl.Limit{} = limit, type, joint_name)
       when type in [:revolute, :continuous] do
    {lower, lower_subs} =
      convert_value_with_ref_or_nil(
        limit.lower,
        &Units.to_radians_or_nil/1,
        joint_name,
        [:limits, :lower]
      )

    {upper, upper_subs} =
      convert_value_with_ref_or_nil(
        limit.upper,
        &Units.to_radians_or_nil/1,
        joint_name,
        [:limits, :upper]
      )

    {velocity, velocity_subs} =
      convert_value_with_ref(
        limit.velocity,
        &Units.to_radians_per_second/1,
        joint_name,
        [:limits, :velocity]
      )

    {effort, effort_subs} =
      convert_value_with_ref(
        limit.effort,
        &Units.to_newton_meters/1,
        joint_name,
        [:limits, :effort]
      )

    limits = %{lower: lower, upper: upper, velocity: velocity, effort: effort}
    subs = lower_subs ++ upper_subs ++ velocity_subs ++ effort_subs
    {limits, subs}
  end

  defp convert_limits(%Dsl.Limit{} = limit, :prismatic, joint_name) do
    {lower, lower_subs} =
      convert_value_with_ref_or_nil(
        limit.lower,
        &Units.to_meters_or_nil/1,
        joint_name,
        [:limits, :lower]
      )

    {upper, upper_subs} =
      convert_value_with_ref_or_nil(
        limit.upper,
        &Units.to_meters_or_nil/1,
        joint_name,
        [:limits, :upper]
      )

    {velocity, velocity_subs} =
      convert_value_with_ref(
        limit.velocity,
        &Units.to_meters_per_second/1,
        joint_name,
        [:limits, :velocity]
      )

    {effort, effort_subs} =
      convert_value_with_ref(limit.effort, &Units.to_newton/1, joint_name, [:limits, :effort])

    limits = %{lower: lower, upper: upper, velocity: velocity, effort: effort}
    subs = lower_subs ++ upper_subs ++ velocity_subs ++ effort_subs
    {limits, subs}
  end

  defp convert_limits(%Dsl.Limit{} = limit, _type, joint_name) do
    {velocity, velocity_subs} =
      convert_value_with_ref(
        limit.velocity,
        &Units.to_radians_per_second/1,
        joint_name,
        [:limits, :velocity]
      )

    {effort, effort_subs} =
      convert_value_with_ref(
        limit.effort,
        &Units.to_newton_meters/1,
        joint_name,
        [:limits, :effort]
      )

    limits = %{lower: nil, upper: nil, velocity: velocity, effort: effort}
    subs = velocity_subs ++ effort_subs
    {limits, subs}
  end

  defp convert_value_with_ref_or_nil(nil, _converter, _joint_name, _field_path), do: {nil, []}

  defp convert_value_with_ref_or_nil(value, converter, joint_name, field_path) do
    convert_value_with_ref(value, converter, joint_name, field_path)
  end

  defp convert_dynamics(nil, _type, _joint_name), do: {nil, []}

  defp convert_dynamics(%Dsl.Dynamics{} = dynamics, type, joint_name)
       when type in [:revolute, :continuous] do
    {damping, damping_subs} =
      convert_value_with_ref_or_nil(
        dynamics.damping,
        &Units.to_rotational_damping_or_nil/1,
        joint_name,
        [:dynamics, :damping]
      )

    {friction, friction_subs} =
      convert_value_with_ref_or_nil(
        dynamics.friction,
        &Units.to_newton_meters_or_nil/1,
        joint_name,
        [:dynamics, :friction]
      )

    converted = %{damping: damping, friction: friction}
    subs = damping_subs ++ friction_subs
    {converted, subs}
  end

  defp convert_dynamics(%Dsl.Dynamics{} = dynamics, type, joint_name)
       when type in [:prismatic, :planar] do
    {damping, damping_subs} =
      convert_value_with_ref_or_nil(
        dynamics.damping,
        &Units.to_linear_damping_or_nil/1,
        joint_name,
        [:dynamics, :damping]
      )

    {friction, friction_subs} =
      convert_value_with_ref_or_nil(
        dynamics.friction,
        &Units.to_newtons_or_nil/1,
        joint_name,
        [:dynamics, :friction]
      )

    converted = %{damping: damping, friction: friction}
    subs = damping_subs ++ friction_subs
    {converted, subs}
  end

  defp convert_dynamics(%Dsl.Dynamics{}, _type, _joint_name) do
    {nil, []}
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
