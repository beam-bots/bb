# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Kinematics.Defn do
  @moduledoc """
  Forward kinematics expressed as a single composable `defn`.

  `BB.Robot.Kinematics` packs a chain's static structure (joint origins, axes
  and types) into plain tensors once, then calls `fk_chain/6` to walk the chain
  in one fused computation rather than dozens of eager per-op `BB.Math` calls.

  Keeping the whole chain walk in `defn` is the point of beam-bots/bb#147: the
  computation can be JIT-compiled and, with a leading batch axis on the inputs,
  vectorised across many joint configurations or many targets at once.

  ## Tensor layout

  For a chain of `n` joints (root-most first), all inputs are `:f64`:

  - `positions` — `{n}` joint positions (radians for revolute, metres for prismatic)
  - `origin_rpy` — `{n, 3}` per-joint origin orientation as `{roll, pitch, yaw}`
  - `origin_xyz` — `{n, 3}` per-joint origin translation
  - `axes` — `{n, 3}` per-joint motion axis (unit vector)
  - `is_revolute` — `{n}` `1.0` for revolute/continuous joints, else `0.0`
  - `is_prismatic` — `{n}` `1.0` for prismatic joints, else `0.0`
  - `stored` — `{n, 4, 4}` per-joint multi-DoF motion, identity for single-DoF joints
  - `deltas` — `{n, 6}` per-joint local perturbation, always passed as zeros

  The result is the `{4, 4}` base-to-tip homogeneous transform. Each joint
  contributes

      origin · scalar_motion(q) · stored · (I + hat(delta))

  where `origin = Rx · Ry · Rz · T(xyz)` reproduces
  `BB.Math.Transform.from_origin/1`, and `scalar_motion` is a Rodrigues rotation
  about `axis` (revolute) or a translation along `axis` (prismatic).

  ## Why multi-DoF joints arrive as a matrix and a zero perturbation

  A `:floating` joint's configuration is a `BB.Math.Transform` and a `:planar`
  joint's is a `BB.Math.Transform2D` lifted into one. Neither can be handed to
  this kernel as scalars without decomposing the rotation into three angles about
  three axes — an Euler decomposition, which is lossy, non-unique and
  gimbal-locked. So the matrix arrives verbatim in `stored`, and forward
  kinematics is bit-exact.

  The Jacobian still needs a differentiable parameter, which is what `deltas` is.
  It is **only ever evaluated at zero**, and that makes the first-order factor
  `I + hat(delta)` exactly right for both jobs:

  - at zero it *is* the identity, so it contributes nothing to forward kinematics;
  - `exp(delta) = I + hat(delta) + O(delta²)`, so its derivative at zero matches
    the true exponential's. The dropped terms are never evaluated.

  Differentiating at the identity also means an Euler chart's singularities are
  never visited, so there is no gimbal lock to regularise — and unlike a genuine
  `se(3)` exponential there is no `(1 - cos θ)/θ²` to blow up at `theta = 0`.

  A single-DoF joint carries `stored = I` and `delta = 0`, so its motion reduces
  to the scalar form. A multi-DoF joint carries both masks at `0.0`, so its
  `scalar_motion` is the identity and its motion reduces to `stored`. One code
  path serves both.
  """

  import Nx.Defn

  @doc """
  Walk a kinematic chain, returning the `{4, 4}` base-to-tip transform.

  See the module documentation for the tensor layout.
  """
  defn fk_chain(
         positions,
         origin_rpy,
         origin_xyz,
         axes,
         is_revolute,
         is_prismatic,
         stored,
         deltas
       ) do
    chain_product(
      joint_matrices(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas
      )
    )
  end

  @doc """
  Compute every link's base-frame transform via a topological prefix-product scan.

  One row per link, ordered root-first so a link's parent always precedes it
  (`parent_idx[i] < i` for every non-root link). `parent_idx` indexes into this
  same ordering; the root carries its own index and an identity joint transform,
  so it resolves to the identity. The per-joint inputs follow the same layout as
  `fk_chain/6`, describing each link's parent joint (identity-valued for the
  root). Returns `{n, 4, 4}`, one transform per link in input order.
  """
  defn link_transforms(
         positions,
         origin_rpy,
         origin_xyz,
         axes,
         is_revolute,
         is_prismatic,
         stored,
         deltas,
         parent_idx
       ) do
    joint_mats =
      joint_matrices(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas
      )

    n = Nx.axis_size(joint_mats, 0)
    init = Nx.broadcast(Nx.eye(4, type: :f64), {n, 4, 4})

    {result, _joint_mats, _parent_idx, _i} =
      while {acc = init, jm = joint_mats, parents = parent_idx, i = 0}, i < n do
        link_transform = Nx.dot(acc[parents[i]], jm[i])
        {Nx.put_slice(acc, [i, 0, 0], Nx.new_axis(link_transform, 0)), jm, parents, i + 1}
      end

    result
  end

  @doc """
  Position Jacobian of the chain tip with respect to each single-DoF joint position.

  Computed by differentiating `fk_chain/8`'s tip translation via `grad` — the
  composable-`defn` payoff #147 is after: no finite differences, no extra
  forward-kinematics evaluations. Inputs follow the `fk_chain/8` layout. Returns
  `{3, n}`: row = spatial axis (x, y, z), column = chain joint in input order.

  Multi-DoF joints get a zero column here, since their motion does not depend on
  `positions`. Their columns come from `position_jacobian_deltas/8`.
  """
  defn position_jacobian(
         positions,
         origin_rpy,
         origin_xyz,
         axes,
         is_revolute,
         is_prismatic,
         stored,
         deltas
       ) do
    args = {origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, stored, deltas}

    Nx.stack([
      position_gradient(positions, args, select_x()),
      position_gradient(positions, args, select_y()),
      position_gradient(positions, args, select_z())
    ])
  end

  @doc """
  Position Jacobian of the chain tip with respect to each joint's local perturbation.

  Returns `{3, n, 6}`: row = spatial axis, then chain joint in input order, then
  the six components of that joint's local perturbation — three translations
  followed by three rotations, in the frame the joint's motion leaves behind.

  A single-DoF joint's block is meaningless and is discarded by the caller; only
  `:planar` and `:floating` joints draw their columns from here, projected onto
  the degrees of freedom they actually have. See the module documentation for why
  differentiating at `deltas = 0` is exact.
  """
  defn position_jacobian_deltas(
         positions,
         origin_rpy,
         origin_xyz,
         axes,
         is_revolute,
         is_prismatic,
         stored,
         deltas
       ) do
    args = {origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, stored, positions}

    Nx.stack([
      delta_gradient(deltas, args, select_x()),
      delta_gradient(deltas, args, select_y()),
      delta_gradient(deltas, args, select_z())
    ])
  end

  @doc """
  Orientation (angular-velocity) Jacobian of the chain tip.

  Returns `{3, n}` where column `j` is the joint's rotation axis expressed in
  the base frame (`z_j`) for revolute/continuous joints, and zero for prismatic
  or fixed joints — the standard geometric angular Jacobian. Stacked beneath the
  position Jacobian it forms the `{6, n}` spatial Jacobian, paired with a
  base-frame rotation-vector orientation error.

  Inputs follow the `fk_chain/6` layout. A prefix-product scan walks the chain
  accumulating the transform up to each joint's axis frame; `grad` is not
  involved, so the data-dependent `while` is fine here.
  """
  defn orientation_jacobian(
         positions,
         origin_rpy,
         origin_xyz,
         axes,
         is_revolute,
         is_prismatic,
         stored,
         deltas
       ) do
    {axes_in_base, _rotations} =
      orientation_frames(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas
      )

    Nx.transpose(axes_in_base * Nx.new_axis(is_revolute, 1))
  end

  @doc """
  Orientation Jacobian of the chain tip with respect to each joint's local perturbation.

  Returns `{3, n, 6}`, matching `position_jacobian_deltas/8`'s layout. A
  perturbation's three translation components produce no angular velocity, so
  those blocks are zero; its three rotation components produce the columns of the
  rotation taking the joint's post-motion frame into the base frame.

  That frame is the right one because the perturbation is applied *after* the
  joint's origin and stored motion — see the module documentation.
  """
  defn orientation_jacobian_deltas(
         positions,
         origin_rpy,
         origin_xyz,
         axes,
         is_revolute,
         is_prismatic,
         stored,
         deltas
       ) do
    {_axes_in_base, rotations} =
      orientation_frames(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas
      )

    n = Nx.axis_size(rotations, 0)
    zeros = Nx.broadcast(Nx.tensor(0.0, type: :f64), {n, 3, 3})

    # `rotations` is {n, 3, 3} with the frame's basis vectors as columns, which is
    # already {spatial, dof} per joint. Concatenating zeros for the translation
    # half gives {n, 3, 6}, then transposing lifts the spatial axis to the front.
    Nx.transpose(Nx.concatenate([zeros, rotations], axis: 2), axes: [1, 0, 2])
  end

  # Walks the chain once, returning both the per-joint rotation axis in the base
  # frame (for the revolute angular Jacobian) and the rotation of each joint's
  # post-motion frame (which is the frame a local perturbation acts in). `grad` is
  # not involved, so the data-dependent `while` is fine here.
  defnp orientation_frames(
          positions,
          origin_rpy,
          origin_xyz,
          axes,
          is_revolute,
          is_prismatic,
          stored,
          deltas
        ) do
    origins = build_origins(origin_rpy, origin_xyz)

    joint_mats =
      joint_matrices(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas
      )

    n = Nx.axis_size(origins, 0)

    {axes_in_base, rotations, _og, _jm, _ax, _prefix, _i} =
      while {acc = Nx.broadcast(Nx.tensor(0.0, type: :f64), {n, 3}),
             rots = Nx.broadcast(Nx.tensor(0.0, type: :f64), {n, 3, 3}), og = origins,
             jm = joint_mats, ax = axes, prefix = Nx.eye(4, type: :f64), i = 0},
            i < n do
        axis_frame = Nx.dot(prefix, og[i])
        z = Nx.dot(Nx.slice(axis_frame, [0, 0], [3, 3]), ax[i])

        post = Nx.dot(prefix, jm[i])
        rotation = Nx.slice(post, [0, 0], [3, 3])

        {Nx.put_slice(acc, [i, 0], Nx.new_axis(z, 0)),
         Nx.put_slice(rots, [i, 0, 0], Nx.new_axis(rotation, 0)), og, jm, ax, post, i + 1}
      end

    {axes_in_base, rotations}
  end

  # The tip translation is `fk · [0, 0, 0, 1]ᵀ` (the homogeneous last column);
  # dotting with a one-hot selector picks one coordinate as a scalar. Done with
  # matmul/dot only — `grad` mishandles range/integer tensor indexing.
  defnp tip_coordinate(
          positions,
          origin_rpy,
          origin_xyz,
          axes,
          is_revolute,
          is_prismatic,
          stored,
          deltas,
          selector
        ) do
    fk =
      fk_chain(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas
      )

    translation = Nx.dot(fk, Nx.tensor([0.0, 0.0, 0.0, 1.0], type: :f64))
    Nx.dot(translation, selector)
  end

  # `origin · scalar_motion(q) · stored · (I + hat(delta))` per joint.
  defnp joint_matrices(
          positions,
          origin_rpy,
          origin_xyz,
          axes,
          is_revolute,
          is_prismatic,
          stored,
          deltas
        ) do
    origins = build_origins(origin_rpy, origin_xyz)
    motions = build_motions(positions, axes, is_revolute, is_prismatic)

    origins
    |> batched_matmul(motions)
    |> batched_matmul(stored)
    |> batched_matmul(augment(deltas))
  end

  # `I + hat(delta)`, the first-order rigid motion. Exactly the identity at
  # `delta = 0`, and its derivative there matches `exp(delta)`'s — which is all
  # that is asked of it, since `delta` is never evaluated anywhere else.
  defnp augment(deltas) do
    vx = deltas[[.., 0]]
    vy = deltas[[.., 1]]
    vz = deltas[[.., 2]]
    wx = deltas[[.., 3]]
    wy = deltas[[.., 4]]
    wz = deltas[[.., 5]]

    z = vx * 0.0
    o = z + 1.0

    Nx.stack(
      [
        Nx.stack([o, -wz, wy, vx], axis: 1),
        Nx.stack([wz, o, -wx, vy], axis: 1),
        Nx.stack([-wy, wx, o, vz], axis: 1),
        Nx.stack([z, z, z, o], axis: 1)
      ],
      axis: 1
    )
  end

  # The three grads are written out rather than mapped: `Enum.map/2` cannot be
  # called inside `defn`, and the chain length is static at trace time anyway.
  defnp position_gradient(positions, args, selector) do
    {origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, stored, deltas} = args

    grad(
      positions,
      &tip_coordinate(
        &1,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        deltas,
        selector
      )
    )
  end

  defnp delta_gradient(deltas, args, selector) do
    {origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, stored, positions} = args

    grad(
      deltas,
      &tip_coordinate(
        positions,
        origin_rpy,
        origin_xyz,
        axes,
        is_revolute,
        is_prismatic,
        stored,
        &1,
        selector
      )
    )
  end

  defnp(select_x, do: Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64))
  defnp(select_y, do: Nx.tensor([0.0, 1.0, 0.0, 0.0], type: :f64))
  defnp(select_z, do: Nx.tensor([0.0, 0.0, 1.0, 0.0], type: :f64))

  defnp build_origins(rpy, xyz) do
    roll = rpy[[.., 0]]
    pitch = rpy[[.., 1]]
    yaw = rpy[[.., 2]]

    rotation =
      batched_matmul(batched_matmul(rotation_x(roll), rotation_y(pitch)), rotation_z(yaw))

    translation = batched_matvec(rotation, xyz)

    homogeneous(rotation, translation)
  end

  defnp build_motions(positions, axes, is_revolute, is_prismatic) do
    angle = positions * is_revolute
    distance = positions * is_prismatic

    rotation = rodrigues(axes, angle)
    translation = axes * Nx.new_axis(distance, 1)

    homogeneous(rotation, translation)
  end

  defnp rotation_x(angle) do
    c = Nx.cos(angle)
    s = Nx.sin(angle)
    z = angle * 0.0
    o = z + 1.0

    stack3(
      o,
      z,
      z,
      z,
      c,
      -s,
      z,
      s,
      c
    )
  end

  defnp rotation_y(angle) do
    c = Nx.cos(angle)
    s = Nx.sin(angle)
    z = angle * 0.0
    o = z + 1.0

    stack3(
      c,
      z,
      s,
      z,
      o,
      z,
      -s,
      z,
      c
    )
  end

  defnp rotation_z(angle) do
    c = Nx.cos(angle)
    s = Nx.sin(angle)
    z = angle * 0.0
    o = z + 1.0

    stack3(
      c,
      -s,
      z,
      s,
      c,
      z,
      z,
      z,
      o
    )
  end

  defnp rodrigues(axes, angle) do
    ax = axes[[.., 0]]
    ay = axes[[.., 1]]
    az = axes[[.., 2]]

    c = Nx.cos(angle)
    s = Nx.sin(angle)
    t = 1.0 - c

    stack3(
      t * ax * ax + c,
      t * ax * ay - s * az,
      t * ax * az + s * ay,
      t * ax * ay + s * az,
      t * ay * ay + c,
      t * ay * az - s * ax,
      t * ax * az - s * ay,
      t * ay * az + s * ax,
      t * az * az + c
    )
  end

  defnp stack3(m00, m01, m02, m10, m11, m12, m20, m21, m22) do
    Nx.stack(
      [
        Nx.stack([m00, m01, m02], axis: 1),
        Nx.stack([m10, m11, m12], axis: 1),
        Nx.stack([m20, m21, m22], axis: 1)
      ],
      axis: 1
    )
  end

  defnp homogeneous(rotation, translation) do
    n = Nx.axis_size(rotation, 0)

    top = Nx.concatenate([rotation, Nx.new_axis(translation, 2)], axis: 2)
    bottom = Nx.broadcast(Nx.tensor([0.0, 0.0, 0.0, 1.0], type: :f64), {n, 1, 4})

    Nx.concatenate([top, bottom], axis: 1)
  end

  defnp batched_matmul(a, b) do
    Nx.dot(a, [2], [0], b, [1], [0])
  end

  defnp batched_matvec(matrices, vectors) do
    Nx.dot(matrices, [2], [0], vectors, [1], [0])
  end

  # Unrolled rather than a data-dependent `while`: the chain length is a static
  # dimension at trace time, so this emits a plain sequence of matmuls. That
  # keeps the product differentiable — `grad` (used for the Jacobian) misroutes
  # through a `while` that dynamically gathers `mats[i]`.
  deftransform chain_product(mats) do
    last = Nx.axis_size(mats, 0) - 1

    Enum.reduce(1..last//1, mats[0], fn i, acc -> Nx.dot(acc, mats[i]) end)
  end
end
