# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Transmission.Resolver do
  @moduledoc """
  Resolve a joint's transmission against the runtime parameter store.

  Transmissions can carry `BB.Dsl.ParamRef` values for any of their fields
  (`reduction`, `offset`, `reversed?`). At compile time those fields appear
  as `nil` on the optimised joint struct and the parameter paths are
  recorded in `BB.Robot.param_subscriptions`. This module fills the nil
  fields in with current parameter values at process startup and
  re-resolves when the parameter changes.
  """

  alias BB.PubSub
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState
  alias BB.Robot.Units

  @type field :: :reduction | :offset | :reversed?
  @type subscriptions :: %{field() => [atom()]}

  @doc """
  Resolve a joint's transmission for use by a server process and subscribe
  to parameter changes for any parameterised fields.

  Returns `{resolved_transmission_or_nil, subscriptions_map}`. The
  subscriptions map maps each transmission field that came from a `param/1`
  reference to its parameter path; the empty map means everything was a
  literal at compile time.

  When the joint has no transmission, returns `{nil, %{}}` and subscribes
  to nothing.
  """
  @spec resolve_and_subscribe(module(), atom()) ::
          {BB.Transmission.t() | nil, subscriptions()}
  def resolve_and_subscribe(robot_module, joint_name) do
    robot = robot_module.robot()

    case robot.joints[joint_name].transmission do
      nil ->
        {nil, %{}}

      transmission ->
        subscriptions = collect_subscriptions(robot.param_subscriptions, joint_name)
        robot_state = Runtime.get_robot_state(robot_module)
        resolved = resolve_fields(transmission, subscriptions, robot, joint_name, robot_state)

        for path <- Map.values(subscriptions) do
          PubSub.subscribe(robot_module, [:param | path])
        end

        {resolved, subscriptions}
    end
  end

  @doc """
  Re-resolve the transmission after a parameter change.

  Call from a server's `handle_info` when a `[:param | path]` message
  arrives. Returns `{:changed, new_transmission}` if the change affects
  this transmission, or `:ignored` otherwise.
  """
  @spec handle_change(
          [atom()],
          BB.Transmission.t(),
          subscriptions(),
          module(),
          atom()
        ) :: {:changed, BB.Transmission.t()} | :ignored
  def handle_change(param_path, current, subscriptions, robot_module, joint_name) do
    if param_path in Map.values(subscriptions) do
      robot = robot_module.robot()
      robot_state = Runtime.get_robot_state(robot_module)

      new_transmission =
        resolve_fields(current, subscriptions, robot, joint_name, robot_state)

      {:changed, new_transmission}
    else
      :ignored
    end
  end

  defp collect_subscriptions(param_subscriptions, joint_name) do
    Enum.flat_map(param_subscriptions, fn {param_path, locations} ->
      Enum.flat_map(locations, fn
        {:joint, ^joint_name, [:transmission, field]} -> [{field, param_path}]
        _ -> []
      end)
    end)
    |> Enum.into(%{})
  end

  defp resolve_fields(transmission, subscriptions, robot, joint_name, robot_state) do
    joint_type = robot.joints[joint_name].type

    Enum.reduce(subscriptions, transmission, fn {field, path}, acc ->
      {:ok, raw_value} = RobotState.get_parameter(robot_state, path)
      Map.put(acc, field, convert_for_field(field, raw_value, joint_type))
    end)
  end

  defp convert_for_field(:reduction, value, _joint_type) when is_number(value),
    do: value * 1.0

  defp convert_for_field(:reversed?, value, _joint_type) when is_boolean(value), do: value

  defp convert_for_field(:offset, %Localize.Unit{} = value, type)
       when type in [:revolute, :continuous],
       do: Units.to_radians(value)

  defp convert_for_field(:offset, %Localize.Unit{} = value, :prismatic),
    do: Units.to_meters(value)

  defp convert_for_field(:offset, %Localize.Unit{} = value, _),
    do: Units.to_radians(value)
end
