# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Transmission.Resolver do
  @moduledoc """
  Resolve an attachment's transmission against the runtime parameter store.

  Transmissions are properties of individual attachments — actuators and
  joint-attached sensors. Their fields (`reduction`, `offset`, `reversed?`)
  may be literal values or `BB.Dsl.ParamRef` references. At compile time the
  parameterised fields appear as `nil` on the optimised attachment info and
  the parameter paths are recorded in `BB.Robot.param_subscriptions`. This
  module fills the `nil` fields in with current parameter values at process
  startup and re-resolves when the parameter changes.
  """

  alias BB.PubSub
  alias BB.Robot
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState
  alias BB.Robot.Units

  @type kind :: :actuator | :sensor
  @type field :: :reduction | :offset | :reversed?
  @type subscriptions :: %{field() => [atom()]}

  @doc """
  Resolve an attachment's transmission against the current parameter store
  without subscribing to changes.

  Suitable for one-shot lookups from helpers that don't run as a long-lived
  process (e.g. `BB.Actuator.publish_begin_motion/3`). Returns the resolved
  transmission, or `nil` if the attachment has no transmission.
  """
  @spec resolve(module(), kind(), atom()) :: BB.Transmission.t() | nil
  def resolve(robot_module, kind, attachment_name) do
    robot = robot_module.robot()

    case attachment_transmission(robot, kind, attachment_name) do
      nil ->
        nil

      transmission ->
        subscriptions = collect_subscriptions(robot.param_subscriptions, kind, attachment_name)
        robot_state = Runtime.get_robot_state(robot_module)
        joint_type = joint_type_for_attachment(robot, kind, attachment_name)
        resolve_fields(transmission, subscriptions, joint_type, robot_state)
    end
  end

  @doc """
  Resolve an attachment's transmission for use by a server process and
  subscribe to parameter changes for any parameterised fields.

  Returns `{resolved_transmission_or_nil, subscriptions_map}`. The
  subscriptions map maps each transmission field that came from a `param/1`
  reference to its parameter path; the empty map means everything was a
  literal at compile time.

  When the attachment has no transmission, returns `{nil, %{}}` and
  subscribes to nothing.
  """
  @spec resolve_and_subscribe(module(), kind(), atom()) ::
          {BB.Transmission.t() | nil, subscriptions()}
  def resolve_and_subscribe(robot_module, kind, attachment_name) do
    robot = robot_module.robot()

    case attachment_transmission(robot, kind, attachment_name) do
      nil ->
        {nil, %{}}

      transmission ->
        subscriptions = collect_subscriptions(robot.param_subscriptions, kind, attachment_name)
        robot_state = Runtime.get_robot_state(robot_module)
        joint_type = joint_type_for_attachment(robot, kind, attachment_name)
        resolved = resolve_fields(transmission, subscriptions, joint_type, robot_state)

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
          kind(),
          atom()
        ) :: {:changed, BB.Transmission.t()} | :ignored
  def handle_change(param_path, current, subscriptions, robot_module, kind, attachment_name) do
    if param_path in Map.values(subscriptions) do
      robot = robot_module.robot()
      robot_state = Runtime.get_robot_state(robot_module)
      joint_type = joint_type_for_attachment(robot, kind, attachment_name)

      new_transmission =
        resolve_fields(current, subscriptions, joint_type, robot_state)

      {:changed, new_transmission}
    else
      :ignored
    end
  end

  defp attachment_transmission(robot, :actuator, name) do
    case robot.actuators[name] do
      %{transmission: transmission} -> transmission
      _ -> nil
    end
  end

  defp attachment_transmission(robot, :sensor, name) do
    case robot.sensors[name] do
      %{transmission: transmission} -> transmission
      _ -> nil
    end
  end

  defp joint_type_for_attachment(robot, :actuator, name) do
    case robot.actuators[name] do
      %{joint: joint_name} ->
        case Robot.get_joint(robot, joint_name) do
          %{type: type} -> type
          _ -> nil
        end

      _ ->
        nil
    end
  end

  defp joint_type_for_attachment(robot, :sensor, name) do
    case robot.sensors[name] do
      %{attached_to: {:joint, joint_name}} ->
        case Robot.get_joint(robot, joint_name) do
          %{type: type} -> type
          _ -> nil
        end

      _ ->
        nil
    end
  end

  defp collect_subscriptions(param_subscriptions, kind, attachment_name) do
    Enum.flat_map(param_subscriptions, fn {param_path, locations} ->
      Enum.flat_map(locations, fn
        {^kind, ^attachment_name, [:transmission, field]} -> [{field, param_path}]
        _ -> []
      end)
    end)
    |> Enum.into(%{})
  end

  defp resolve_fields(transmission, subscriptions, joint_type, robot_state) do
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
