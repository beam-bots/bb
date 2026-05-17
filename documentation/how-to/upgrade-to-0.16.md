<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Upgrading to bb 0.16

bb 0.16 introduces two breaking changes. The fastest path forwards is:

```bash
mix igniter.upgrade bb
```

That runs the `bb.upgrade` Igniter task, which applies the mechanical migrations and prints a notice listing the remaining manual follow-ups. The sections below describe each change so you can audit the diff or migrate by hand if you don't use Igniter.

## 1. `auto_disarm_on_error` removed; topology supervisor escalates instead

The old `auto_disarm_on_error: true` default disarmed the whole robot whenever any component called `BB.Safety.report_error/3`. That contradicted the fault-isolation philosophy of the supervision tree — a single overheating servo would stop everything.

0.16 replaces that with supervisor-driven escalation:

* All hardware-facing subsystems now run under a new `BB.TopologySupervisor` with its own restart budget.
* If failures cascade up and exhaust that budget, the topology supervisor stops and `BB.Safety.Controller` force-disarms the robot.
* `BB.Safety.report_error/3` is now notification-only — it publishes a `BB.Safety.HardwareError` event on `[:safety, :error]` and returns. It does **not** change safety state.

### Removed

```elixir
settings do
  auto_disarm_on_error false   # ← delete this line
end
```

The upgrader removes the line for you. If the `settings do … end` block ends up empty after the removal, you can delete the whole block.

### New tunables

If you want to tune how much failure your robot will absorb before force-disarm, set them in the robot DSL:

```elixir
settings do
  topology_max_restarts 3   # default
  topology_max_seconds 5    # default
end
```

### Migrating code that called `report_error/3` expecting disarm

If you had components calling `BB.Safety.report_error/3` and relying on the side-effect of auto-disarm, two options:

1. **Crash to escalate (preferred).** Raise or exit when you detect an unrecoverable fault — the supervisor will restart you, and repeated restarts naturally hit the topology supervisor's budget. The escalation is then automatic and observable.

   ```elixir
   # In an actuator on persistent communication failure:
   BB.Safety.report_error(robot, path, :servo_unreachable)
   raise BB.Error.Hardware.Unreachable, path: path
   ```

2. **Subscribe and disarm explicitly.** If you want a softer response in certain situations, subscribe to `[:safety, :error]` and call `BB.Safety.disarm/1` from your own logic when appropriate.

## 2. `ex_cldr_units` replaced with `localize`

bb's unit handling now uses Kip Cole's [`localize`](https://hex.pm/packages/localize) package, which consolidates the `ex_cldr_*` family into a single dependency with no compile-time backend module.

### What the upgrader rewrites automatically

In any module that uses a BB DSL macro (`use BB`, `use BB.Actuator`, `use BB.Sensor`, `use BB.Controller`, `use BB.Bridge`, `use BB.Command`) or implements the `BB.Safety` behaviour:

| Before | After |
|---|---|
| `Cldr.Unit.new!(:meter, 5)` | `Localize.Unit.new!(5, "meter")` |
| `Cldr.Unit.new!(5, :meter)` | `Localize.Unit.new!(5, "meter")` |
| `Cldr.Unit.convert!(unit, :radian)` | `Localize.Unit.convert!(unit, "radian")` |
| `Cldr.Unit.compatible?(u, :meter)` | `Localize.Unit.compatible?(u, "meter")` |
| `Cldr.Unit.compare(a, b)` | `Localize.Unit.compare(a, b)` |
| `Cldr.Unit.to_string!(unit)` | `Localize.Unit.to_string!(unit)` |
| `%Cldr.Unit{unit: :meter, value: 1}` | `%Localize.Unit{name: "meter", value: 1}` |

It also rewrites `alias BB.Cldr.Unit` to `alias BB.Unit` across the codebase (the `BB.Cldr` backend module no longer exists).

### What the upgrader can't safely automate

* **`.unit` field access on bare variables.** Code like `do_something(my_unit.unit)` needs to become `my_unit.name` and the value is now a string, not an atom. The upgrader doesn't rewrite this because there's no reliable way to know which `.unit` accesses are on unit structs.
* **Custom Cldr backend modules.** If you defined your own `MyApp.Cldr` backend with `Cldr.Unit` as a provider, the upgrader doesn't touch it. Whether you keep it (because your app uses `ex_cldr_units` for its own reasons) or remove it is your call.
* **`BB.Safety.report_error/3` callers** — the function still exists and the call sites still compile. The behaviour change (no longer auto-disarms) is described above and needs to be addressed by hand.

### Unit name format

bb's `~u(...)` sigil keeps accepting atom-style names with underscores (`~u(10 newton_meter)`); they're translated to CLDR canonical dash form (`"newton-meter"`) inside `BB.Unit`. So existing DSL definitions and most sigil call sites need no change.

When you write CLDR unit identifiers directly (e.g. in a `Localize.Unit.new!/2` call), use the dash form: `"newton-meter"`, not `"newton_meter"`.

## Verifying the upgrade

After running the upgrader (or doing the migration by hand):

```bash
mix compile --warnings-as-errors
mix test
```

If you have downstream bb packages (servo drivers, custom sensors, etc.), bump their `bb` constraint to `~> 0.16` and re-run their checks too.
