# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Defn do
  @moduledoc """
  Composable `defn` numerical kernels operating on raw Nx tensors.

  This is the computational core of the math layer. The struct wrappers
  (`BB.Math.Vec3`, `BB.Math.Quaternion`, `BB.Math.Transform`) delegate their
  hot operations here, and higher-level algorithms (forward kinematics,
  Jacobians, IK iterations) are expected to compose these kernels inside their
  own `defn` rather than threading values back through eager per-op struct
  calls.

  Keeping the maths in `defn` is the point: a `defn` can be JIT-compiled,
  vectorised over a leading batch axis, and composed into a larger computation,
  none of which is possible for eager `Nx.*` calls dispatched one operation at
  a time.

  All inputs and outputs are raw `:f64` tensors with these conventions:

  - quaternions are `{4}` tensors in WXYZ (scalar-first) order
  - transforms are `{4, 4}` homogeneous matrices
  """

  import Nx.Defn

  @doc """
  Hamilton product of two quaternions, returning a normalised unit quaternion.

  `quaternion_multiply(q1, q2)` composes the rotations: `q2` is applied first,
  then `q1`. Inputs and output are `{4}` WXYZ tensors.
  """
  defn quaternion_multiply(q1, q2) do
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    w2 = q2[0]
    x2 = q2[1]
    y2 = q2[2]
    z2 = q2[3]

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    Nx.stack([w, x, y, z]) |> normalise_quaternion()
  end

  @doc """
  Normalise a `{4}` quaternion tensor to unit length.

  Falls back to the identity quaternion when the input is near-zero, so the
  result is always a valid unit rotation.
  """
  defn normalise_quaternion(q) do
    norm = Nx.LinAlg.norm(q)
    identity = Nx.tensor([1.0, 0.0, 0.0, 0.0], type: :f64)

    Nx.select(norm > 1.0e-10, q / norm, identity)
  end

  @doc """
  Compose two `{4, 4}` homogeneous transforms.

  `transform_compose(a, b)` returns the transform that applies `a` first, then
  `b`.
  """
  defn transform_compose(a, b) do
    Nx.dot(a, b)
  end
end
