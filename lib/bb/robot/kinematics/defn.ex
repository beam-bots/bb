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

  The result is the `{4, 4}` base-to-tip homogeneous transform. The per-joint
  transform reproduces `BB.Math.Transform.from_origin/1` composed with the
  motion transform: `origin = Rx · Ry · Rz · T(xyz)` and the motion is a
  Rodrigues rotation about `axis` (revolute) or a translation along `axis`
  (prismatic). Fixed/floating/planar joints carry both masks at `0.0`, leaving
  an identity motion.
  """

  import Nx.Defn

  @doc """
  Walk a kinematic chain, returning the `{4, 4}` base-to-tip transform.

  See the module documentation for the tensor layout.
  """
  defn fk_chain(positions, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic) do
    origins = build_origins(origin_rpy, origin_xyz)
    motions = build_motions(positions, axes, is_revolute, is_prismatic)
    chain_product(batched_matmul(origins, motions))
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
         parent_idx
       ) do
    origins = build_origins(origin_rpy, origin_xyz)
    motions = build_motions(positions, axes, is_revolute, is_prismatic)
    joint_mats = batched_matmul(origins, motions)

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
  Position Jacobian of the chain tip with respect to the chain joint positions.

  Computed by differentiating `fk_chain/6`'s tip translation via `grad` — the
  composable-`defn` payoff #147 is after: no finite differences, no extra
  forward-kinematics evaluations. Inputs follow the `fk_chain/6` layout. Returns
  `{3, n}`: row = spatial axis (x, y, z), column = chain joint in input order.
  """
  defn position_jacobian(positions, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic) do
    select_x = Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64)
    select_y = Nx.tensor([0.0, 1.0, 0.0, 0.0], type: :f64)
    select_z = Nx.tensor([0.0, 0.0, 1.0, 0.0], type: :f64)

    jx =
      grad(
        positions,
        &tip_coordinate(&1, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, select_x)
      )

    jy =
      grad(
        positions,
        &tip_coordinate(&1, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, select_y)
      )

    jz =
      grad(
        positions,
        &tip_coordinate(&1, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic, select_z)
      )

    Nx.stack([jx, jy, jz])
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
          selector
        ) do
    fk = fk_chain(positions, origin_rpy, origin_xyz, axes, is_revolute, is_prismatic)
    translation = Nx.dot(fk, Nx.tensor([0.0, 0.0, 0.0, 1.0], type: :f64))
    Nx.dot(translation, selector)
  end

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
