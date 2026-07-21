<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Anti-patterns

The mistakes assistants make most often with BB. Each has a correct form in the
matching sub-rule.

### Don't command a disarmed robot

A new robot is `:disarmed` and ignores motion. Arm it first through the command
system:

```elixir
{:ok, cmd} = MyRobot.Robot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)
BB.Actuator.set_position!(MyRobot.Robot, :servo, 0.5)
```

### Don't manipulate the safety system directly

```elixir
# Bad — skips the user's prearm checks
BB.Safety.arm(MyRobot.Robot)

# Good — the Arm command runs prearm checks
{:ok, cmd} = MyRobot.Robot.arm()
```

### Don't use bare numbers for physical quantities in the DSL

```elixir
limit lower: -1.57, upper: 1.57                        # Bad — ambiguous units
limit lower: ~u(-90 degree), upper: ~u(90 degree)      # Good
```

### Don't hand-roll supervision

```elixir
Supervisor.start_link([MyServo], strategy: :one_for_one)   # Bad
MyRobot.Robot.start_link()                                 # Good — BB builds the tree
```

### Don't pass the robot module where kinematics wants the struct

```elixir
# Bad — Kinematics operates on %BB.Robot{}, not the module
BB.Robot.Kinematics.link_position(MyRobot.Robot, positions, :tip)

# Good
BB.Robot.Kinematics.link_position(MyRobot.Robot.robot(), positions, :tip)
```

### Don't guess API names

Current, easily-confused signatures:

- Current joint positions: `BB.Robot.Runtime.positions/1` (not
  `joint_positions/1`).
- Command callback is `handle_command/3` (goal, context, state) — not `/2`.
- IK goes through `BB.Motion.move_to/4` with a required `:solver:`; there is no
  default solver in core.

When unsure, `mix usage_rules.search_docs "<topic>" -p bb` rather than
inventing a function.
