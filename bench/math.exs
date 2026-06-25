# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

# Baseline costs for the small fixed-size math operations. These are the
# operations #147 accepts will stay relatively expensive per-call (eager
# dispatch / JIT overhead on tiny tensors); the suite exists to measure that
# cost, and to give a before/after when the operations move into `defn`.
#
# Run with: mix run bench/math.exs

alias BB.Math.Quaternion
alias BB.Math.Transform
alias BB.Math.Vec3

q1 = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 3)
q2 = Quaternion.from_axis_angle(Vec3.unit_x(), :math.pi() / 4)

t1 = Transform.from_axis_angle(Vec3.unit_z(), :math.pi() / 3)
t2 = Transform.translation(Vec3.new(0.1, 0.2, 0.3))

chain = for i <- 1..7, do: Transform.from_axis_angle(Vec3.unit_z(), i * 0.1)

Benchee.run(
  %{
    "Quaternion.multiply/2" => fn -> Quaternion.multiply(q1, q2) end,
    "Transform.compose/2" => fn -> Transform.compose(t1, t2) end,
    "Transform.compose_all/1 (7-link chain)" => fn -> Transform.compose_all(chain) end
  },
  warmup: 1,
  time: 3
)
