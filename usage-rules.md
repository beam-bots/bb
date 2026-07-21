<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Beam Bots (BB) Usage Rules

BB is a framework for building resilient robots in Elixir. You describe a
robot's physical structure with a Spark DSL (`use BB`); BB compiles that to a
struct and generates a supervision tree that mirrors the robot, so a crash is
isolated to the affected subtree rather than taking the whole robot down.

Read the documentation before reaching for a feature. Do not assume prior
knowledge of BB — its DSL, its safety model, and its message conventions are
specific and easy to guess wrong. Search it with
`mix usage_rules.search_docs "<topic>" -p bb` and consult
[hexdocs](https://hexdocs.pm/bb) rather than inventing an API.

## Golden rules

1. **A robot is a module that does `use BB`.** By convention it is named
   `MyRobot.Robot`. Never hand-roll GenServers or a `Supervisor` for a robot's
   components — declare them in the DSL and let BB build the tree.
2. **Physical quantities use the `~u` sigil, never bare numbers:**
   `~u(90 degree)`, not `1.57`. `use BB` brings the sigil into scope. Values are
   converted to SI base units (metres, radians, kg) in the compiled struct.
3. **A robot starts `:disarmed` and ignores motion commands until armed.**
   Arming runs user-defined prearm checks. Change the safety state through the
   command system, never by calling `BB.Safety` directly.
4. **Talk to a running robot through its public API** — `BB.PubSub`,
   `BB.Actuator`, `BB.Parameter`, `BB.Motion`, and the command functions
   generated onto the robot module — not by messaging its processes yourself.

## Sub-rules

Consult the focused rules for the area you are working in:

| Sub-rule | Covers |
|---|---|
| `bb:dsl-topology` | Defining a robot: links, joints, units, attaching components |
| `bb:safety-and-commands` | The arm/disarm contract, the state machine, writing commands |
| `bb:pubsub-and-sensors` | Subscribing to and publishing messages; message naming |
| `bb:actuators` | Commanding motion; joint-space vs motor-space |
| `bb:kinematics` | Forward kinematics and inverse kinematics |
| `bb:parameters` | Runtime-adjustable configuration and bridges |
| `bb:simulation` | Running a robot without hardware |
| `bb:anti-patterns` | The mistakes assistants make most often |

Pull them into your project with `mix usage_rules.sync <file> bb:all`.

## Further reading

- [Your First Robot](https://hexdocs.pm/bb/01-first-robot.html) — start here
- [DSL reference](https://hexdocs.pm/bb/dsl-bb.html)
- [Understanding Safety](https://hexdocs.pm/bb/understanding-safety.html)
