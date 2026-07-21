<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Parameters

Parameters are validated, runtime-adjustable configuration values with change
notifications — think ROS2 parameters. Declare them in a `parameters` section;
`group` nests them into a path.

```elixir
parameters do
  param :max_speed, type: :float, default: 1.0,
    min: 0.0, max: 10.0, doc: "Maximum velocity in m/s"

  group :motion do
    param :gain, type: :float, default: 0.5
  end
end
```

Each `param` needs a `type` (`:float`, `:integer`, `:boolean`, `:string`,
`:atom`, or `{:unit, unit_type}` for physical quantities) and a `default`.
`min`/`max` bound numeric types.

## Reading and writing at runtime

Address a parameter by its **path** (a list reflecting the group nesting):

```elixir
{:ok, speed} = BB.Parameter.get(MyRobot.Robot, [:max_speed])
1.0 = BB.Parameter.get!(MyRobot.Robot, [:max_speed])
:ok = BB.Parameter.set(MyRobot.Robot, [:motion, :gain], 0.8)
:ok = BB.Parameter.set_many(MyRobot.Robot, [{[:max_speed], 2.0}, {[:motion, :gain], 0.9}])
```

`set/3` validates against the schema and returns `{:error, reason}` on invalid
values; unknown paths return `{:error, :not_found}`.

## Reacting to changes

A change is more than a stored value — it propagates two ways:

- **Component options that reference a parameter update live.** Reference one in
  a DSL declaration with `param([...])`; when it changes, BB re-resolves the
  options and calls the component's `handle_options/2` callback with the new
  values. This is the idiomatic way for a sensor, actuator, controller, or
  estimator to track configuration — do **not** hand-wire your own subscription
  inside the component for this.

  ```elixir
  controller :bus, {MyController, port: param([:config, :port])}

  # in MyController:
  def handle_options(new_opts, state), do: {:ok, reconfigure(state, new_opts)}
  ```

- **Every change is published on `[:param | path]`.** Any process can
  `BB.PubSub.subscribe(MyRobot.Robot, [:param])` (or a narrower path) to receive
  change notifications directly — this is how bridges mirror parameters to
  external systems.

## Startup and bridges

Pass overrides at start (`MyRobot.Robot.start_link(params: [max_speed: 2.0])`);
invalid or unknown values make the supervised robot fail to boot. A `bridge`
declaration exposes parameters bidirectionally to an external system (a servo
bus, a ground station).

See [Parameters](https://hexdocs.pm/bb/07-parameters.html) and
[Parameter Bridges](https://hexdocs.pm/bb/08-parameter-bridges.html).
