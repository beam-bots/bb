<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Kinematics

## Forward kinematics — "where is this link?"

`BB.Robot.Kinematics` works on the **compiled `%BB.Robot{}` struct**, not the
robot module. Get the struct with `MyRobot.Robot.robot()`, and current joint
positions from the runtime with `BB.Robot.Runtime.positions/1`:

```elixir
robot = MyRobot.Robot.robot()
positions = BB.Robot.Runtime.positions(MyRobot.Robot)   # %{joint_name => radians}

# Cartesian position of a link:
{x, y, z} = BB.Robot.Kinematics.link_position(robot, positions, :camera_link)

# Full 4x4 pose (position + orientation):
transform = BB.Robot.Kinematics.forward_kinematics(robot, positions, :camera_link)

# Every link at once (more efficient than repeated calls):
transforms = BB.Robot.Kinematics.all_link_transforms(robot, positions)
```

You can also pass an explicit `%{joint => radians}` map instead of the live
positions to ask "where *would* this link be". Transforms are
`BB.Math.Transform` 4x4 matrices; angles are radians throughout.

## Inverse kinematics — "what joint angles reach this point?"

IK solvers are **pluggable** and ship in satellite packages (`bb_ik_dls`,
`bb_ik_fabrik`) implementing the `BB.IK.Solver` behaviour. Drive them through
`BB.Motion`, which solves, updates state, and commands the actuators:

```elixir
{:ok, meta} =
  BB.Motion.move_to(MyRobot.Robot, :gripper, {0.3, 0.2, 0.1}, solver: BB.IK.FABRIK)
```

- **`:solver` is required** — core ships no default. Add a solver package and
  pass its module.
- Targets are `{x, y, z}` in metres.
- Use `BB.Motion.solve_only/4` to compute angles without moving.
- Solver options (`:max_iterations`, `:tolerance`, `:respect_limits`) are
  passed through untyped; defaults differ between solvers, so set them
  explicitly when it matters.

See [Forward Kinematics](https://hexdocs.pm/bb/04-kinematics.html) and
[Inverse Kinematics](https://hexdocs.pm/bb/09-inverse-kinematics.html).
