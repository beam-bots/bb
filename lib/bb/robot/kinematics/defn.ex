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

  defnp chain_product(mats) do
    n = Nx.axis_size(mats, 0)

    {result, _mats, _i} =
      while {acc = Nx.eye(4, type: :f64), m = mats, i = 0}, i < n do
        {Nx.dot(acc, m[i]), m, i + 1}
      end

    result
  end
end
