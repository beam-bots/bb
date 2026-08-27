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
  given the current joint configurations:

      # Get the transform from base to end-effector
      transform = BB.Robot.Kinematics.forward_kinematics(
        robot,
        state,
        :end_effector
      )

      # Extract position
      pos = BB.Math.Transform.get_translation(transform)
      {BB.Math.Vec3.x(pos), BB.Math.Vec3.y(pos), BB.Math.Vec3.z(pos)}

  ## Multi-DoF joints

  A joint's configuration is shaped to its type: a bare float for single-DoF
  joints, a `BB.Math.Transform2D` for `:planar`, a `BB.Math.Transform` for
  `:floating`. See `BB.Robot.State` for the full table. A multi-DoF joint's
  transform is used verbatim, so forward kinematics through a floating base is
  bit-exact.

  This makes **Jacobian width the sum of degrees of freedom along the chain**
  rather than the number of joints in it — a floating joint contributes six
  columns and a planar one three. `jacobian_columns/2` reports which joint and
  degree of freedom each column belongs to.

  ## Conventions

  - All positions are in meters
  - All angles are in radians
  - Transforms are 4x4 homogeneous matrices (Nx tensors)
  - The base link is at the identity transform
  """

  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.Joint
  alias BB.Robot.Kinematics.Defn
  alias BB.Robot.State

  @typedoc """
  A joint's configuration, shaped to its type.

  See `BB.Robot.State` for the table of which shape belongs to which joint type.
  """
  @type configuration :: State.configuration()

  @typedoc "A map of joint configurations, as `BB.Robot.State` returns."
  @type configurations :: %{atom() => configuration()}

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
      BB.Robot.State.set_configuration(state, :shoulder, :math.pi() / 4)

      transform = BB.Robot.Kinematics.forward_kinematics(robot, state, :forearm)
      pos = BB.Math.Transform.get_translation(transform)
  """
  @spec forward_kinematics(Robot.t(), State.t() | configurations(), atom()) :: Transform.t()
  def forward_kinematics(%Robot{} = robot, %State{} = state, target_link) do
    positions = State.get_all_configurations(state)
    forward_kinematics(robot, positions, target_link)
  end

  def forward_kinematics(%Robot{} = robot, positions, target_link) when is_map(positions) do
    compute_chain_transform(robot, positions, path_to!(robot, target_link))
  end

  @doc """
  Compute transforms for all links in the robot.

  Returns a map from link name to its transform in the base frame.

  ## Examples

      transforms = BB.Robot.Kinematics.all_link_transforms(robot, state)
      end_effector_transform = transforms[:end_effector]
  """
  @spec all_link_transforms(Robot.t(), State.t() | configurations()) ::
          %{atom() => Transform.t()}
  def all_link_transforms(%Robot{} = robot, %State{} = state) do
    positions = State.get_all_configurations(state)
    all_link_transforms(robot, positions)
  end

  def all_link_transforms(%Robot{} = robot, positions) when is_map(positions) do
    # `link_order` is root-first, so a link's parent is always already resolved
    # by the time the link itself is reached — the same property the batched
    # prefix-product scan in `Defn.link_transforms/9` relies on. The root has no
    # parent joint and resolves to the identity.
    Enum.reduce(robot.topology.link_order, %{}, fn link_name, transforms ->
      {:ok, link} = Robot.get_link(robot, link_name)

      Map.put(transforms, link_name, link_transform(robot, positions, transforms, link))
    end)
  end

  defp link_transform(_robot, _positions, _transforms, %{parent_joint: nil}) do
    Transform.identity()
  end

  defp link_transform(robot, positions, transforms, %{parent_joint: joint_name}) do
    joint = Map.fetch!(robot.joints, joint_name)

    transforms
    |> Map.fetch!(joint.parent_link)
    |> Transform.compose(joint_transform(positions, joint))
  end

  @doc """
  Get the position of a link in the base frame.

  This is a convenience function that extracts just the translation
  from the forward kinematics transform.

  ## Examples

      {x, y, z} = BB.Robot.Kinematics.link_position(robot, state, :end_effector)
  """
  @spec link_position(Robot.t(), State.t() | configurations(), atom()) ::
          {float(), float(), float()}
  def link_position(%Robot{} = robot, state_or_positions, target_link) do
    transform = forward_kinematics(robot, state_or_positions, target_link)
    pos = Transform.get_translation(transform)
    {Vec3.x(pos), Vec3.y(pos), Vec3.z(pos)}
  end

  @doc """
  Compute the position Jacobian of a link with respect to the given joints.

  Returns a `{3, columns}` tensor where each column is the partial derivative of
  the link's base-frame position with respect to one degree of freedom. Joints
  that do not lie on the chain to `target_link` (and so do not move it) get zero
  columns.

  **Width is the sum of degrees of freedom over `joint_names`, not their count** —
  a `:floating` joint contributes six columns and a `:planar` one three, while a
  `:fixed` joint contributes none. Use `jacobian_columns/2` to find out which
  joint and degree of freedom each column belongs to.

  Computed analytically by differentiating the forward-kinematics `defn`, rather
  than by finite differences.

  ## Examples

      jacobian = BB.Robot.Kinematics.position_jacobian(robot, configurations, :tool0, joint_names)
  """
  @spec position_jacobian(Robot.t(), State.t() | configurations(), atom(), [atom()]) ::
          Nx.Tensor.t()
  def position_jacobian(%Robot{} = robot, %State{} = state, target_link, joint_names) do
    configurations = State.get_all_configurations(state)
    position_jacobian(robot, configurations, target_link, joint_names)
  end

  def position_jacobian(%Robot{} = robot, configurations, target_link, joint_names)
      when is_map(configurations) do
    chain_joints = chain_joints(robot, path_to!(robot, target_link))
    chain_jacobian(robot, configurations, chain_joints, joint_names, :position)
  end

  @doc """
  Compute the spatial (position and orientation) Jacobian of a link.

  Returns a `{6, columns}` tensor: the top three rows are the position Jacobian
  (see `position_jacobian/4`) and the bottom three are the orientation Jacobian.
  For a revolute joint the orientation column is its rotation axis in the base
  frame; for a multi-DoF joint's rotational degrees of freedom it is the
  corresponding axis of the frame the joint's motion leaves behind; for prismatic
  and purely translational degrees of freedom it is zero. This pairs with an
  orientation error expressed as a base-frame rotation vector.

  As with `position_jacobian/4`, width is the sum of degrees of freedom over
  `joint_names` — see `jacobian_columns/2`.

  ## Examples

      jacobian = BB.Robot.Kinematics.jacobian(robot, configurations, :tool0, joint_names)
  """
  @spec jacobian(Robot.t(), State.t() | configurations(), atom(), [atom()]) :: Nx.Tensor.t()
  def jacobian(%Robot{} = robot, %State{} = state, target_link, joint_names) do
    configurations = State.get_all_configurations(state)
    jacobian(robot, configurations, target_link, joint_names)
  end

  def jacobian(%Robot{} = robot, configurations, target_link, joint_names)
      when is_map(configurations) do
    chain_joints = chain_joints(robot, path_to!(robot, target_link))

    Nx.concatenate(
      [
        chain_jacobian(robot, configurations, chain_joints, joint_names, :position),
        chain_jacobian(robot, configurations, chain_joints, joint_names, :orientation)
      ],
      axis: 0
    )
  end

  @doc """
  Compute the transform for a single joint given its current position.

  This combines the joint's fixed origin transform with the variable
  transform due to joint motion.
  """
  @spec compute_joint_transform(Robot.t(), %{atom() => configuration()}, atom()) :: Transform.t()
  def compute_joint_transform(%Robot{} = robot, configurations, joint_name) do
    {:ok, joint} = Robot.get_joint(robot, joint_name)
    configuration = Map.get(configurations, joint_name)

    Transform.compose(
      Transform.from_origin(joint.origin),
      motion_transform(joint, configuration)
    )
  end

  @doc """
  Describe the columns a Jacobian over `joint_names` will have.

  Jacobian width is the sum of degrees of freedom along the chain rather than the
  number of joints, so a caller applying a solver's delta needs to know which
  joint and which degree of freedom each column belongs to. Returns one
  `{joint_name, dof_index}` per column, in column order. Fixed joints contribute
  nothing.

  A `:planar` joint's three degrees of freedom are, in order, its two in-plane
  translations and its rotation about the plane normal — the same order as its
  `BB.Math.Transform2D` configuration. A `:floating` joint's six are three
  translations then three rotations, in the frame its motion leaves behind.

  ## Examples

      BB.Robot.Kinematics.jacobian_columns(robot, [:base, :mast])
      #=> [{:base, 0}, {:base, 1}, {:base, 2}, {:mast, 0}]
  """
  @spec jacobian_columns(Robot.t(), [atom()]) :: [{atom(), non_neg_integer()}]
  def jacobian_columns(%Robot{} = robot, joint_names) do
    Enum.flat_map(joint_names, fn joint_name ->
      case Joint.dof(get_joint!(robot, joint_name)) do
        0 -> []
        dof -> Enum.map(0..(dof - 1), &{joint_name, &1})
      end
    end)
  end

  defp motion_transform(%{type: type} = joint, configuration)
       when type in [:revolute, :continuous] do
    Transform.from_axis_angle(joint_axis(joint), scalar(configuration))
  end

  defp motion_transform(%{type: :prismatic} = joint, configuration) do
    Transform.translation_along(joint_axis(joint), scalar(configuration))
  end

  defp motion_transform(%{type: :planar} = joint, %Transform2D{} = configuration) do
    Transform2D.to_transform(configuration, joint_axis(joint))
  end

  defp motion_transform(%{type: :floating}, %Transform{} = configuration), do: configuration

  # An absent or wrongly-shaped configuration means "at rest", which is the
  # identity. `BB.Robot.State.set_configuration/3` is what rejects a bad shape;
  # forward kinematics is also called with bare maps a caller assembled by hand.
  defp motion_transform(_joint, _configuration), do: Transform.identity()

  defp joint_axis(%{axis: axis}), do: tuple_to_vec3(axis || {0.0, 0.0, 1.0})

  defp scalar(value) when is_number(value), do: value * 1.0
  defp scalar(_), do: 0.0

  defp tuple_to_vec3({x, y, z}), do: Vec3.new(x, y, z)

  # These entry points raise rather than return a result tuple, which is the
  # contract they have always had. Raising the structured error keeps the
  # diagnosis in one place.
  defp path_to!(robot, target_link) do
    case Robot.path_to(robot, target_link) do
      {:ok, path} -> path
      {:error, error} -> raise error
    end
  end

  # Naming a joint the robot does not have used to yield one zero column, which
  # silently swallowed a typo. It cannot survive per-DoF columns anyway: there is
  # no way to know how many columns a joint that doesn't exist should contribute.
  defp get_joint!(robot, joint_name) do
    case Robot.get_joint(robot, joint_name) do
      {:ok, joint} -> joint
      {:error, error} -> raise error
    end
  end

  defp chain_joints(%Robot{joints: joints}, path) do
    Enum.filter(path, &Map.has_key?(joints, &1))
  end

  defp compute_chain_transform(%Robot{} = robot, configurations, path) do
    robot
    |> chain_joints(path)
    |> Enum.map(&joint_transform(configurations, Map.fetch!(robot.joints, &1)))
    |> Transform.compose_all()
  end

  # The scalar counterpart of one row of `Defn.joint_matrices/8`: the joint's
  # origin, then its motion. `motion_transform/2` already covers both halves the
  # `defn` keeps apart — the masked scalar motion of a single-DoF joint, and the
  # `stored` matrix a multi-DoF one carries.
  #
  # Forward kinematics walks a chain of at most a few dozen links and wants a
  # single transform back. Pushing that through the batched `defn` costs far more
  # in trace and dispatch than the ~64 multiply-adds per link it performs, so it
  # composes here instead. The Jacobians still go through `defn`: they are
  # differentiated with `grad`, which has no scalar counterpart, and they
  # genuinely benefit from the batch axis.
  #
  # `Defn.joint_matrices/8` also applies `augment(delta)`, which is the identity
  # at `delta = 0`. Only the Jacobians ever pass a non-zero delta, so there is
  # nothing to mirror here.
  defp joint_transform(configurations, joint) do
    joint.origin
    |> origin_transform()
    |> Transform.compose(motion_transform(joint, Map.get(configurations, joint.name)))
  end

  # `Rx * Ry * Rz` with the translation carried through it, matching
  # `Defn.build_origins/2`, whose translation is `rotation * xyz` rather than a
  # bare `xyz`.
  defp origin_transform(%{orientation: {roll, pitch, yaw}, position: {x, y, z}}) do
    Transform.rotation_x(roll)
    |> Transform.compose(Transform.rotation_y(pitch))
    |> Transform.compose(Transform.rotation_z(yaw))
    |> Transform.compose(Transform.translation(Vec3.new(x, y, z)))
  end

  defp origin_transform(_origin), do: Transform.identity()

  defp chain_jacobian(robot, configurations, chain_joints, joint_names, kind) do
    case chain_joints do
      [] ->
        Nx.broadcast(Nx.tensor(0.0, type: :f64), {3, column_count(robot, joint_names)})

      _ ->
        tensors = chain_tensors(robot, configurations, chain_joints)

        assemble_columns(
          robot,
          apply(Defn, scalar_fun(kind), tensors),
          apply(Defn, delta_fun(kind), tensors),
          chain_joints,
          joint_names
        )
    end
  end

  defp column_count(robot, joint_names) do
    Enum.sum_by(joint_names, &Joint.dof(get_joint!(robot, &1)))
  end

  defp scalar_fun(:position), do: :position_jacobian
  defp scalar_fun(:orientation), do: :orientation_jacobian
  defp delta_fun(:position), do: :position_jacobian_deltas
  defp delta_fun(:orientation), do: :orientation_jacobian_deltas

  defp chain_tensors(robot, configurations, chain_joints) do
    joints = Enum.map(chain_joints, &Map.fetch!(robot.joints, &1))

    [
      rows(joints, &scalar_position(configurations, &1)),
      rows(joints, &origin_rpy(&1.origin)),
      rows(joints, &origin_xyz(&1.origin)),
      rows(joints, &axis_row(&1.axis)),
      rows(joints, &revolute_mask/1),
      rows(joints, &prismatic_mask/1),
      stored_motions(configurations, joints),
      Nx.broadcast(Nx.tensor(0.0, type: :f64), {length(joints), 6})
    ]
  end

  # A multi-DoF joint's motion cannot be expressed as a scalar, so it travels in
  # `stored` as a matrix instead and its scalar slot is zeroed — which, with both
  # motion masks also zero, leaves its scalar motion as the identity.
  defp scalar_position(configurations, joint) do
    case Map.get(configurations, joint.name, 0.0) do
      value when is_number(value) -> value * 1.0
      _ -> 0.0
    end
  end

  defp stored_motions(configurations, joints) do
    joints
    |> Enum.map(&stored_motion(Map.get(configurations, &1.name), &1))
    |> Nx.stack()
  end

  defp stored_motion(%Transform2D{} = configuration, %{type: :planar} = joint) do
    configuration
    |> Transform2D.to_transform(plane_normal(joint))
    |> Transform.tensor()
  end

  defp stored_motion(%Transform{} = configuration, %{type: :floating}),
    do: Transform.tensor(configuration)

  defp stored_motion(_configuration, _joint), do: Nx.eye(4, type: :f64)

  defp plane_normal(%{axis: axis}), do: tuple_to_vec3(axis || {0.0, 0.0, 1.0})

  # Columns are per degree of freedom, not per joint, so a floating joint
  # contributes six and a planar one three. Single-DoF joints take their column
  # from the scalar gradient; multi-DoF joints project their {3, 6} perturbation
  # block onto the degrees of freedom they actually have.
  defp assemble_columns(robot, scalar_jacobian, delta_jacobian, chain_joints, joint_names) do
    index_of = chain_joints |> Enum.with_index() |> Map.new()

    joint_names
    |> Enum.flat_map(fn joint_name ->
      joint = get_joint!(robot, joint_name)

      joint_columns(
        joint,
        Map.fetch(index_of, joint_name),
        scalar_jacobian,
        delta_jacobian
      )
    end)
    |> Nx.stack(axis: 1)
  end

  defp joint_columns(%{type: :fixed}, _index, _scalar_jacobian, _delta_jacobian), do: []

  # A joint that isn't on the chain doesn't move the target, so it gets a zero
  # column per degree of freedom rather than being dropped — the caller asked for
  # it, and the column count must match what `jacobian_columns/2` reports.
  defp joint_columns(joint, :error, _scalar_jacobian, _delta_jacobian) do
    List.duplicate(Nx.broadcast(Nx.tensor(0.0, type: :f64), {3}), Joint.dof(joint))
  end

  defp joint_columns(%{type: type}, {:ok, index}, scalar_jacobian, _delta_jacobian)
       when type in [:revolute, :continuous, :prismatic] do
    [scalar_jacobian[[.., index]]]
  end

  defp joint_columns(joint, {:ok, index}, _scalar_jacobian, delta_jacobian) do
    delta_jacobian[[.., index, ..]]
    |> Nx.dot(dof_basis(joint))
    |> split_columns()
  end

  defp split_columns(tensor) do
    Enum.map(0..(Nx.axis_size(tensor, 1) - 1), &tensor[[.., &1]])
  end

  # Maps a joint's own degrees of freedom onto the six components of a local
  # perturbation. A floating joint uses all of them; a planar joint uses the two
  # in-plane translations and the rotation about its normal, in the same basis
  # `BB.Math.Transform2D.to_transform/2` lifts its configuration through.
  defp dof_basis(%{type: :floating}), do: Nx.eye(6, type: :f64)

  defp dof_basis(%{type: :planar} = joint) do
    normal = Vec3.normalise(plane_normal(joint))
    {u, v} = Transform2D.plane_basis(normal)

    Nx.stack(
      [
        Nx.concatenate([Vec3.tensor(u), Nx.broadcast(Nx.tensor(0.0, type: :f64), {3})]),
        Nx.concatenate([Vec3.tensor(v), Nx.broadcast(Nx.tensor(0.0, type: :f64), {3})]),
        Nx.concatenate([Nx.broadcast(Nx.tensor(0.0, type: :f64), {3}), Vec3.tensor(normal)])
      ],
      axis: 1
    )
  end

  defp rows(joints, fun), do: joints |> Enum.map(fun) |> Nx.tensor(type: :f64)

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
