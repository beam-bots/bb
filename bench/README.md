<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Benchmarks

Baseline measurements for the numerical layer, supporting the move to
composable `defn` kernels (beam-bots/bb#147) and the benchmarking ask in
beam-bots/bb#149.

Run a suite with `mix run`:

```bash
mix run bench/math.exs   # quaternion multiply, transform compose, chain compose
mix run bench/fk.exs     # forward kinematics on the 6-DOF example arm
```

`bench/fk.exs` uses `BB.ExampleRobots`, which is compiled from `test/support`
in the `dev` and `test` environments, so a plain `mix run` works.

## Scope

These cover what the `bb` package itself owns: the `BB.Math` operations and the
forward-kinematics chain walk. Jacobian and IK-iteration benchmarks live with
their implementations in `bb_ik_dls` and are added there.

Batched/multi-target forward kinematics is not yet benchmarked — it depends on
forward kinematics becoming a single `defn` (a later phase of #147). The suite
is structured so those cases slot in once that work lands.
