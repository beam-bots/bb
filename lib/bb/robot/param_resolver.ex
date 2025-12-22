# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.ParamResolver do
  @moduledoc """
  Resolves parameter references in robot structs.

  When a robot's DSL uses `param([:path, :to, :param])` instead of literal unit
  values, the Builder stores `nil` for those fields and records subscriptions
  in `param_subscriptions`. This module handles:

  1. **Initial resolution** - At startup, resolve all param refs using current
     parameter values
  2. **Dynamic updates** - When a parameter changes, update all affected fields
     in the robot struct
  """

  alias BB.Robot
  alias BB.Robot.{State, Units}

  @doc """
  Resolve all parameter references in a robot struct.

  Iterates through `robot.param_subscriptions` and resolves each parameter
  reference using the current value from `robot_state`.

  Returns the updated robot struct with all param refs resolved to values.
  """
  @spec resolve_all(Robot.t(), State.t()) :: Robot.t()
  def resolve_all(%Robot{param_subscriptions: subs} = robot, _robot_state)
      when map_size(subs) == 0 do
    robot
  end

  def resolve_all(%Robot{param_subscriptions: subs} = robot, robot_state) do
    Enum.reduce(subs, robot, fn {param_path, locations}, robot ->
      case State.get_parameter(robot_state, param_path) do
        {:ok, value} ->
          update_locations(robot, locations, value)

        {:error, :not_found} ->
          robot
      end
    end)
  end

  @doc """
  Update all fields that reference a specific parameter.

  When a parameter changes, this function updates all robot struct fields
  that reference that parameter path.

  Returns the updated robot struct.
  """
  @spec update_for_param(Robot.t(), [atom()], term(), State.t()) :: Robot.t()
  def update_for_param(%Robot{param_subscriptions: subs} = robot, param_path, new_value, _state) do
    case Map.fetch(subs, param_path) do
      {:ok, locations} ->
        update_locations(robot, locations, new_value)

      :error ->
        robot
    end
  end

  defp update_locations(robot, locations, value) do
    Enum.reduce(locations, robot, fn location, robot ->
      update_field(robot, location, value)
    end)
  end

  defp update_field(robot, {:joint, joint_name, field_path}, value) do
    joint = Map.fetch!(robot.joints, joint_name)
    updated_joint = apply_field_update(joint, field_path, value)
    %{robot | joints: Map.put(robot.joints, joint_name, updated_joint)}
  end

  defp apply_field_update(joint, [:origin, :x], value) do
    si_value = Units.to_meters(value)
    update_origin_position(joint, 0, si_value)
  end

  defp apply_field_update(joint, [:origin, :y], value) do
    si_value = Units.to_meters(value)
    update_origin_position(joint, 1, si_value)
  end

  defp apply_field_update(joint, [:origin, :z], value) do
    si_value = Units.to_meters(value)
    update_origin_position(joint, 2, si_value)
  end

  defp apply_field_update(joint, [:origin, :roll], value) do
    si_value = Units.to_radians(value)
    update_origin_orientation(joint, 0, si_value)
  end

  defp apply_field_update(joint, [:origin, :pitch], value) do
    si_value = Units.to_radians(value)
    update_origin_orientation(joint, 1, si_value)
  end

  defp apply_field_update(joint, [:origin, :yaw], value) do
    si_value = Units.to_radians(value)
    update_origin_orientation(joint, 2, si_value)
  end

  defp apply_field_update(joint, [:axis | _rest], _value) do
    # Axis updates require all three values to recompute the axis vector
    # For now, we'll just leave the existing value (nil from Builder)
    # A full implementation would track all axis values and recompute
    joint
  end

  defp apply_field_update(joint, [:limits, :lower], value) do
    si_value = convert_limit_value(joint.type, :lower, value)
    put_in(joint.limits.lower, si_value)
  end

  defp apply_field_update(joint, [:limits, :upper], value) do
    si_value = convert_limit_value(joint.type, :upper, value)
    put_in(joint.limits.upper, si_value)
  end

  defp apply_field_update(joint, [:limits, :velocity], value) do
    si_value = convert_limit_value(joint.type, :velocity, value)
    put_in(joint.limits.velocity, si_value)
  end

  defp apply_field_update(joint, [:limits, :effort], value) do
    si_value = convert_limit_value(joint.type, :effort, value)
    put_in(joint.limits.effort, si_value)
  end

  defp apply_field_update(joint, [:dynamics, :damping], value) do
    si_value = convert_dynamics_value(joint.type, :damping, value)
    put_in(joint.dynamics.damping, si_value)
  end

  defp apply_field_update(joint, [:dynamics, :friction], value) do
    si_value = convert_dynamics_value(joint.type, :friction, value)
    put_in(joint.dynamics.friction, si_value)
  end

  defp update_origin_position(%{origin: nil} = joint, index, value) do
    position = put_elem({0.0, 0.0, 0.0}, index, value)
    %{joint | origin: %{position: position, orientation: {0.0, 0.0, 0.0}}}
  end

  defp update_origin_position(joint, index, value) do
    position = put_elem(joint.origin.position, index, value)
    %{joint | origin: %{joint.origin | position: position}}
  end

  defp update_origin_orientation(%{origin: nil} = joint, index, value) do
    orientation = put_elem({0.0, 0.0, 0.0}, index, value)
    %{joint | origin: %{position: {0.0, 0.0, 0.0}, orientation: orientation}}
  end

  defp update_origin_orientation(joint, index, value) do
    orientation = put_elem(joint.origin.orientation, index, value)
    %{joint | origin: %{joint.origin | orientation: orientation}}
  end

  defp convert_limit_value(type, field, value) when type in [:revolute, :continuous] do
    case field do
      :lower -> Units.to_radians_or_nil(value)
      :upper -> Units.to_radians_or_nil(value)
      :velocity -> Units.to_radians_per_second(value)
      :effort -> Units.to_newton_meters(value)
    end
  end

  defp convert_limit_value(:prismatic, field, value) do
    case field do
      :lower -> Units.to_meters_or_nil(value)
      :upper -> Units.to_meters_or_nil(value)
      :velocity -> Units.to_meters_per_second(value)
      :effort -> Units.to_newton(value)
    end
  end

  defp convert_limit_value(_type, field, value) do
    case field do
      :velocity -> Units.to_radians_per_second(value)
      :effort -> Units.to_newton_meters(value)
      _ -> nil
    end
  end

  defp convert_dynamics_value(type, field, value) when type in [:revolute, :continuous] do
    case field do
      :damping -> Units.to_rotational_damping_or_nil(value)
      :friction -> Units.to_newton_meters_or_nil(value)
    end
  end

  defp convert_dynamics_value(type, field, value) when type in [:prismatic, :planar] do
    case field do
      :damping -> Units.to_linear_damping_or_nil(value)
      :friction -> Units.to_newtons_or_nil(value)
    end
  end

  defp convert_dynamics_value(_type, _field, _value), do: nil
end
