<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Kinematics

## Forward kinematics — "where is this link?"

`BB.Robot.Kinematics` works on the **compiled `%BB.Robot{}` struct**, not the
robot module. Get the struct with `MyRobot.Robot.robot()`, and current joint
configurations from the runtime with `BB.Robot.Runtime.configurations/1`:

```elixir
robot = MyRobot.Robot.robot()
configurations = BB.Robot.Runtime.configurations(MyRobot.Robot)

# Cartesian position of a link:
{x, y, z} = BB.Robot.Kinematics.link_position(robot, configurations, :camera_link)

# Full 4x4 pose (position + orientation):
transform = BB.Robot.Kinematics.forward_kinematics(robot, configurations, :camera_link)

# Every link at once (more efficient than repeated calls):
transforms = BB.Robot.Kinematics.all_link_transforms(robot, configurations)
```

You can also pass an explicit configuration map instead of the live one to ask
"where *would* this link be". A single-DoF joint's configuration is a bare float
— radians for revolute, metres for prismatic — while `:planar` and `:floating`
joints carry a `BB.Math.Transform2D` and a `BB.Math.Transform` respectively. See
`BB.Robot.State` for the full table. Transforms are `BB.Math.Transform` 4x4
matrices; angles are radians throughout.

## Inverse kinematics — "what joint angles reach this point?"

IK solvers are **pluggable** and ship in satellite packages (`bb_ik_dls`,
`bb_ik_fabrik`) implementing the `BB.IK.Solver` behaviour. Drive them through
`BB.Motion`, which solves and commands the actuators:

```elixir
{:ok, meta} =
  BB.Motion.move_to(MyRobot.Robot, :gripper, {0.3, 0.2, 0.1},
    source_link: :base_link,
    solver: BB.IK.FABRIK
  )
```

- **`:solver` is required** — core ships no default. Add a solver package and
  pass its module.
- **`:source_link` is required too**, and has no default. The root is right for a
  fixed-base arm and silently wrong for a robot whose base floats, where it drags
  a 6-DoF joint into a problem that has no business containing one. Pass
  `BB.Robot.root_link(robot)` when you do mean the whole tree.
- Targets are `{x, y, z}` in metres.
- Use `BB.Motion.solve_only/4` to compute angles without moving.
- **Commanding a joint doesn't move it in `BB.Robot.State`.** That table is
  written from `JointState` sensor messages only, because a commanded position
  isn't a measured one — and IK seeds each solve from it. A joint whose
  actuator has no position feedback needs a
  `BB.Sensor.OpenLoopPositionEstimator`, or its configuration never changes and
  every solve starts from the same place. The DSL warns at compile time when a
  driven joint has no sensor; `actuator :servo, Driver, sensor: false` says the
  actuator reports its own position.
- Solver options (`:max_iterations`, `:tolerance`, `:respect_limits`) are
  passed through untyped; defaults differ between solvers, so set them
  explicitly when it matters.

See [Forward Kinematics](https://hexdocs.pm/bb/04-kinematics.html) and
[Inverse Kinematics](https://hexdocs.pm/bb/09-inverse-kinematics.html).
