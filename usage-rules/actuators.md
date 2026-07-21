<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Actuators and Commanding Motion

Send an actuator a target with `BB.Actuator`. The robot must be **armed**
first (see `bb:safety-and-commands`) — commands to a disarmed robot are
ignored.

```elixir
# By full path within the topology ([joint, actuator]), value in radians:
BB.Actuator.set_position(MyRobot.Robot, [:pan_joint, :servo], 0.785)

# By the actuator's unique name (raises on error):
BB.Actuator.set_position!(MyRobot.Robot, :servo, 0.785)
```

The DSL takes `~u` sigil values; the runtime command functions take plain
numbers in SI base units (radians here).

- `set_position/4` takes the **full path** (a list) to the actuator;
  `set_position!/4` takes just the actuator's unique **name**. `set_velocity`
  and `set_effort` follow the same pair.
- Positions are in **radians**, velocities in rad/s — SI base units, the same
  units the compiled robot struct uses.
- Use `set_position_sync/5` when you need to wait for acknowledgement rather
  than fire-and-forget.

## Joint-space in, motor-space handled for you

You command joints in **joint-space**. BB applies the joint's `transmission`
(gearing, `offset`, `reversed?`) and hands the driver **motor-space** values.
By the time a `%BB.Message.Actuator.Command.Position{}` (or `Velocity`,
`Effort`, `Trajectory`) reaches an actuator callback, the numbers are already
in motor-space — the driver does no joint-to-motor maths.

## Writing an actuator

`use BB.Actuator`, define `init/1`, the GenServer callbacks, and the
**required** `disarm/1`. Handle command messages in `handle_cast/2`:

```elixir
alias BB.Message.Actuator.Command

def handle_cast({:command, %BB.Message{payload: %Command.Position{} = cmd}}, state) do
  drive_hardware(cmd, state)
  {:noreply, state}
end

def disarm(opts), do: cut_power(opts)   # must work without GenServer state
```

To report position back in joint-space, either let BB publish for you
(`BB.Actuator.publish_begin_motion/3`) or translate with
`BB.Actuator.to_joint_space/3` and publish yourself.

See [Writing an Actuator](https://hexdocs.pm/bb/12-writing-an-actuator.html).
