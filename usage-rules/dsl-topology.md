<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Defining a Robot: Topology

A robot is a module that does `use BB`. Convention names it `MyRobot.Robot`.
The `topology` section describes the physical structure as a tree of links and
joints; `use BB` also injects a `robot/0` function returning the compiled
`%BB.Robot{}` struct.

```elixir
defmodule MyRobot.Robot do
  use BB

  topology do
    link :base_link do
      joint :pan_joint do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(5 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :servo, {MyDriver, servo_id: 1}

        link :pan_link
      end
    end
  end
end
```

## Writing the DSL

This is a Spark DSL — every entity is a macro. Follow the house style:

- **Omit brackets on DSL calls:** `type :revolute`, not `type(:revolute)`.
- **Each entity takes positional arguments then options, in one of three
  forms:**
  - `entity <positional>, <options as a keyword list>`
  - `entity <positional> do <options as nested calls> end`
  - `entity do <positional and options as nested calls> end`
- **Prefer the keyword-list form** unless the entity contains *nested DSL* that
  needs a `do`/`end` block. So `limit lower: ~u(...), upper: ~u(...)` (plain
  options), but `link`/`joint` take a block because they nest other entities.

## Rules

- **Nesting encodes the kinematic chain.** A `joint` lives inside its parent
  `link`; the child `link` lives inside the `joint`. This nesting *is* the
  chain — there is no separate parent/child field.
- **Joint `type`** is one of `:revolute`, `:prismatic`, `:fixed`,
  `:continuous`, `:floating`, `:planar`.
- **`limit` requires `effort` and `velocity`**; `lower`/`upper`/`acceleration`
  are optional. It has no nested DSL, so use the keyword form.
- **`axis` defaults to the Z-axis** and can be omitted. Reorient it with
  `roll`/`pitch`/`yaw`, e.g. `axis roll: ~u(90 degree)`.
- **Attach components as `{Module, opts}`** in an actuator/sensor slot:
  `actuator :servo, {MyDriver, servo_id: 1}`. A bare `Module` works when it
  needs no options. Names must be unique across the whole robot — BB registers
  each process under its name. Add a `do`/`end` block only to nest further DSL
  such as `transmission`.
- **Per-attachment `transmission`** describes how *that* actuator/sensor maps
  joint-space to its hardware (`reversed?`, `offset`, gearing). It belongs on
  the attachment, not the joint, because several devices can share one joint.

## Units

Every physical value uses the `~u` sigil — `~u(90 degree)`,
`~u(5 newton_meter)`, `~u(60 degree_per_second)`. Unit names are validated
dynamically by the [`localize`](https://hexdocs.pm/localize) package, which is
the authoritative source for valid names — including composite units, which
join their parts with underscores and `per`: `~u(1.0 meter_per_second)`,
`~u(0.5 meter_per_second_squared)`, `~u(1.5 newton_second_per_meter)`. If a
unit is rejected, check `localize` rather than guessing.

Robot-level components not attached to a joint (a GPS, a battery monitor, a bus
controller) go in the top-level `sensors` and `controllers` sections rather
than inside `topology`.

See the [DSL reference](https://hexdocs.pm/bb/dsl-bb.html) and
[Your First Robot](https://hexdocs.pm/bb/01-first-robot.html).
