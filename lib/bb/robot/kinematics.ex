# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Kinematics do
  @moduledoc """
  Kinematic computations for robot manipulators.

  This module provides forward kinematics and related computations
  for robots defined with the BB DSL.

  ## Forward Kinematics

  Forward kinematics computes the position and orientation of any link
  given the current joint positions:

      # Get the transform from base to end-effector
      transform = BB.Robot.Kinematics.forward_kinematics(
        robot,
        state,
        :end_effector
      )

      # Extract position
      pos = BB.Math.Transform.get_translation(transform)
      {BB.Math.Vec3.x(pos), BB.Math.Vec3.y(pos), BB.Math.Vec3.z(pos)}

  ## Conventions

  - All positions are in meters
  - All angles are in radians
  - Transforms are 4x4 homogeneous matrices (Nx tensors)
  - The base link is at the identity transform
  """

  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.Kinematics.Defn
  alias BB.Robot.State

  @doc """
  Compute the forward kinematics transform from base to a target link.

  Returns a 4x4 homogeneous transformation matrix representing the
  position and orientation of the target link in the base frame.

  ## Parameters

  - `robot`: The Robot struct
  - `state`: The current robot state (or a map of joint positions)
  - `target_link`: The name of the link to compute the transform for

  ## Examples

      robot = MyRobot.robot()
      {:ok, state} = BB.Robot.State.new(robot)
      BB.Robot.State.set_joint_position(state, :shoulder, :math.pi() / 4)

      transform = BB.Robot.Kinematics.forward_kinematics(robot, state, :forearm)
      pos = BB.Math.Transform.get_translation(transform)
  """
  @spec forward_kinematics(Robot.t(), State.t() | %{atom() => float()}, atom()) :: Transform.t()
  def forward_kinematics(%Robot{} = robot, %State{} = state, target_link) do
    positions = State.get_all_positions(state)
    forward_kinematics(robot, positions, target_link)
  end

  def forward_kinematics(%Robot{} = robot, positions, target_link) when is_map(positions) do
    path = Robot.path_to(robot, target_link)

    if is_nil(path) do
      raise ArgumentError, "Unknown link: #{inspect(target_link)}"
    end

    compute_chain_transform(robot, positions, path)
  end

  @doc """
  Compute transforms for all links in the robot.

  Returns a map from link name to its transform in the base frame.

  ## Examples

      transforms = BB.Robot.Kinematics.all_link_transforms(robot, state)
      end_effector_transform = transforms[:end_effector]
  """
  @spec all_link_transforms(Robot.t(), State.t() | %{atom() => float()}) ::
          %{atom() => Transform.t()}
  def all_link_transforms(%Robot{} = robot, %State{} = state) do
    positions = State.get_all_positions(state)
    all_link_transforms(robot, positions)
  end

  def all_link_transforms(%Robot{} = robot, positions) when is_map(positions) do
    link_order = robot.topology.link_order
    index_of = link_order |> Enum.with_index() |> Map.new()

    specs =
      Enum.map(link_order, fn link_name ->
        link = Robot.get_link(robot, link_name)
        link_joint_spec(robot, positions, index_of, link_name, link.parent_joint)
      end)

    result =
      Defn.link_transforms(
        col(specs, :position, :f64),
        col(specs, :rpy, :f64),
        col(specs, :xyz, :f64),
        col(specs, :axis, :f64),
        col(specs, :revolute, :f64),
        col(specs, :prismatic, :f64),
        col(specs, :parent_index, :s64)
      )

    link_order
    |> Enum.with_index()
    |> Map.new(fn {link_name, index} -> {link_name, Transform.from_tensor(result[index])} end)
  end

  @doc """
  Get the position of a link in the base frame.

  This is a convenience function that extracts just the translation
  from the forward kinematics transform.

  ## Examples

      {x, y, z} = BB.Robot.Kinematics.link_position(robot, state, :end_effector)
  """
  @spec link_position(Robot.t(), State.t() | %{atom() => float()}, atom()) ::
          {float(), float(), float()}
  def link_position(%Robot{} = robot, state_or_positions, target_link) do
    transform = forward_kinematics(robot, state_or_positions, target_link)
    pos = Transform.get_translation(transform)
    {Vec3.x(pos), Vec3.y(pos), Vec3.z(pos)}
  end

  @doc """
  Compute the position Jacobian of a link with respect to the given joints.

  Returns a `{3, length(joint_names)}` tensor where each column is the partial
  derivative of the link's base-frame position with respect to that joint's
  position. Joints that do not lie on the chain to `target_link` (and so do not
  move it) get a zero column. Columns follow `joint_names` order.

  Computed analytically by differentiating the forward-kinematics `defn`, rather
  than by finite differences.

  ## Examples

      jacobian = BB.Robot.Kinematics.position_jacobian(robot, positions, :tool0, joint_names)
  """
  @spec position_jacobian(Robot.t(), State.t() | %{atom() => float()}, atom(), [atom()]) ::
          Nx.Tensor.t()
  def position_jacobian(%Robot{} = robot, %State{} = state, target_link, joint_names) do
    positions = State.get_all_positions(state)
    position_jacobian(robot, positions, target_link, joint_names)
  end

  def position_jacobian(%Robot{} = robot, positions, target_link, joint_names)
      when is_map(positions) do
    path = Robot.path_to(robot, target_link)

    if is_nil(path) do
      raise ArgumentError, "Unknown link: #{inspect(target_link)}"
    end

    chain_joints = Enum.filter(path, &Map.has_key?(robot.joints, &1))
    chain_jacobian(robot, positions, chain_joints, joint_names)
  end

  @doc """
  Compute the transform for a single joint given its current position.

  This combines the joint's fixed origin transform with the variable
  transform due to joint motion.
  """
  @spec compute_joint_transform(Robot.t(), %{atom() => float()}, atom()) :: Transform.t()
  def compute_joint_transform(%Robot{} = robot, positions, joint_name) do
    joint = Robot.get_joint(robot, joint_name)
    position = Map.get(positions, joint_name, 0.0)

    origin_transform = Transform.from_origin(joint.origin)

    motion_transform =
      case joint.type do
        type when type in [:revolute, :continuous] ->
          axis = tuple_to_vec3(joint.axis || {0.0, 0.0, 1.0})
          Transform.from_axis_angle(axis, position)

        :prismatic ->
          axis = tuple_to_vec3(joint.axis || {0.0, 0.0, 1.0})
          Transform.translation_along(axis, position)

        :fixed ->
          Transform.identity()

        :floating ->
          Transform.identity()

        :planar ->
          Transform.identity()
      end

    Transform.compose(origin_transform, motion_transform)
  end

  defp tuple_to_vec3({x, y, z}), do: Vec3.new(x, y, z)

  defp compute_chain_transform(%Robot{} = robot, positions, path) do
    case Enum.filter(path, &Map.has_key?(robot.joints, &1)) do
      [] ->
        Transform.identity()

      joint_names ->
        joints = Enum.map(joint_names, &Map.fetch!(robot.joints, &1))

        Defn.fk_chain(
          chain_positions(positions, joint_names),
          rows(joints, &origin_rpy(&1.origin)),
          rows(joints, &origin_xyz(&1.origin)),
          rows(joints, &axis_row(&1.axis)),
          column(joints, &revolute_mask/1),
          column(joints, &prismatic_mask/1)
        )
        |> Transform.from_tensor()
    end
  end

  defp chain_jacobian(_robot, _positions, [], joint_names) do
    Nx.broadcast(Nx.tensor(0.0, type: :f64), {3, length(joint_names)})
  end

  defp chain_jacobian(robot, positions, chain_joints, joint_names) do
    joints = Enum.map(chain_joints, &Map.fetch!(robot.joints, &1))

    chain_jac =
      Defn.position_jacobian(
        chain_positions(positions, chain_joints),
        rows(joints, &origin_rpy(&1.origin)),
        rows(joints, &origin_xyz(&1.origin)),
        rows(joints, &axis_row(&1.axis)),
        column(joints, &revolute_mask/1),
        column(joints, &prismatic_mask/1)
      )

    select_columns(chain_jac, chain_joints, joint_names)
  end

  defp select_columns(chain_jac, chain_joints, joint_names) do
    index_of = chain_joints |> Enum.with_index() |> Map.new()
    zero = Nx.broadcast(Nx.tensor(0.0, type: :f64), {3})

    joint_names
    |> Enum.map(fn joint_name ->
      case Map.fetch(index_of, joint_name) do
        {:ok, index} -> chain_jac[[.., index]]
        :error -> zero
      end
    end)
    |> Nx.stack(axis: 1)
  end

  defp chain_positions(positions, joint_names) do
    joint_names
    |> Enum.map(&(Map.get(positions, &1, 0.0) * 1.0))
    |> Nx.tensor(type: :f64)
  end

  defp rows(joints, fun), do: joints |> Enum.map(fun) |> Nx.tensor(type: :f64)
  defp column(joints, fun), do: joints |> Enum.map(fun) |> Nx.tensor(type: :f64)

  defp col(specs, key, type), do: specs |> Enum.map(&Map.fetch!(&1, key)) |> Nx.tensor(type: type)

  defp link_joint_spec(_robot, _positions, index_of, link_name, nil) do
    %{
      rpy: origin_rpy(nil),
      xyz: origin_xyz(nil),
      axis: axis_row(nil),
      revolute: revolute_mask(nil),
      prismatic: prismatic_mask(nil),
      position: 0.0,
      parent_index: Map.fetch!(index_of, link_name)
    }
  end

  defp link_joint_spec(robot, positions, index_of, _link_name, joint_name) do
    joint = Map.fetch!(robot.joints, joint_name)

    %{
      rpy: origin_rpy(joint.origin),
      xyz: origin_xyz(joint.origin),
      axis: axis_row(joint.axis),
      revolute: revolute_mask(joint),
      prismatic: prismatic_mask(joint),
      position: Map.get(positions, joint_name, 0.0) * 1.0,
      parent_index: Map.fetch!(index_of, joint.parent_link)
    }
  end

  defp origin_rpy(%{orientation: {roll, pitch, yaw}}), do: [roll, pitch, yaw]
  defp origin_rpy(_), do: [0.0, 0.0, 0.0]

  defp origin_xyz(%{position: {x, y, z}}), do: [x, y, z]
  defp origin_xyz(_), do: [0.0, 0.0, 0.0]

  defp axis_row({x, y, z}), do: [x, y, z]
  defp axis_row(_), do: [0.0, 0.0, 1.0]

  defp revolute_mask(%{type: type}) when type in [:revolute, :continuous], do: 1.0
  defp revolute_mask(_), do: 0.0

  defp prismatic_mask(%{type: :prismatic}), do: 1.0
  defp prismatic_mask(_), do: 0.0
end
