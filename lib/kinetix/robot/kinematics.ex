# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.Kinematics do
  @moduledoc """
  Kinematic computations for robot manipulators.

  This module provides forward kinematics and related computations
  for robots defined with the Kinetix DSL.

  ## Forward Kinematics

  Forward kinematics computes the position and orientation of any link
  given the current joint positions:

      # Get the transform from base to end-effector
      transform = Kinetix.Robot.Kinematics.forward_kinematics(
        robot,
        state,
        :end_effector
      )

      # Extract position
      {x, y, z} = Kinetix.Robot.Transform.get_translation(transform)

  ## Conventions

  - All positions are in meters
  - All angles are in radians
  - Transforms are 4x4 homogeneous matrices (Nx tensors)
  - The base link is at the identity transform
  """

  alias Kinetix.Robot
  alias Kinetix.Robot.{State, Transform}

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
      {:ok, state} = Kinetix.Robot.State.new(robot)
      Kinetix.Robot.State.set_joint_position(state, :shoulder, :math.pi() / 4)

      transform = Kinetix.Robot.Kinematics.forward_kinematics(robot, state, :forearm)
      {x, y, z} = Kinetix.Robot.Transform.get_translation(transform)
  """
  @spec forward_kinematics(Robot.t(), State.t() | %{atom() => float()}, atom()) :: Nx.Tensor.t()
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

      transforms = Kinetix.Robot.Kinematics.all_link_transforms(robot, state)
      end_effector_transform = transforms[:end_effector]
  """
  @spec all_link_transforms(Robot.t(), State.t() | %{atom() => float()}) ::
          %{atom() => Nx.Tensor.t()}
  def all_link_transforms(%Robot{} = robot, %State{} = state) do
    positions = State.get_all_positions(state)
    all_link_transforms(robot, positions)
  end

  def all_link_transforms(%Robot{} = robot, positions) when is_map(positions) do
    robot.topology.link_order
    |> Enum.reduce(%{}, fn link_name, transforms ->
      transform =
        case Robot.get_link(robot, link_name) do
          %{parent_joint: nil} ->
            Transform.identity()

          %{parent_joint: parent_joint_name} ->
            parent_link = robot.joints[parent_joint_name].parent_link
            parent_transform = Map.fetch!(transforms, parent_link)
            joint_transform = compute_joint_transform(robot, positions, parent_joint_name)
            Transform.compose(parent_transform, joint_transform)
        end

      Map.put(transforms, link_name, transform)
    end)
  end

  @doc """
  Get the position of a link in the base frame.

  This is a convenience function that extracts just the translation
  from the forward kinematics transform.

  ## Examples

      {x, y, z} = Kinetix.Robot.Kinematics.link_position(robot, state, :end_effector)
  """
  @spec link_position(Robot.t(), State.t() | %{atom() => float()}, atom()) ::
          {float(), float(), float()}
  def link_position(%Robot{} = robot, state_or_positions, target_link) do
    transform = forward_kinematics(robot, state_or_positions, target_link)
    Transform.get_translation(transform)
  end

  @doc """
  Compute the transform for a single joint given its current position.

  This combines the joint's fixed origin transform with the variable
  transform due to joint motion.
  """
  @spec compute_joint_transform(Robot.t(), %{atom() => float()}, atom()) :: Nx.Tensor.t()
  def compute_joint_transform(%Robot{} = robot, positions, joint_name) do
    joint = Robot.get_joint(robot, joint_name)
    position = Map.get(positions, joint_name, 0.0)

    origin_transform = Transform.from_origin(joint.origin)

    motion_transform =
      case joint.type do
        type when type in [:revolute, :continuous] ->
          axis = joint.axis || {0.0, 0.0, 1.0}
          Transform.revolute_transform(axis, position)

        :prismatic ->
          axis = joint.axis || {0.0, 0.0, 1.0}
          Transform.prismatic_transform(axis, position)

        :fixed ->
          Transform.identity()

        :floating ->
          Transform.identity()

        :planar ->
          Transform.identity()
      end

    Transform.compose(origin_transform, motion_transform)
  end

  defp compute_chain_transform(%Robot{} = robot, positions, path) do
    path
    |> Enum.filter(&Map.has_key?(robot.joints, &1))
    |> Enum.reduce(Transform.identity(), fn joint_name, acc ->
      joint_transform = compute_joint_transform(robot, positions, joint_name)
      Transform.compose(acc, joint_transform)
    end)
  end
end
