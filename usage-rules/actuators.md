<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Actuators and Commanding Motion

Send an actuator a target with `BB.Actuator`. The robot must be **armed**
first (see `bb:safety-and-commands`) — commands to a disarmed robot are
ignored.

```elixir
# By the actuator's unique name, value in radians:
BB.Actuator.set_position(MyRobot.Robot, :servo, 0.785)

# Or by its full path through the topology ([link, joint, actuator]):
BB.Actuator.set_position(MyRobot.Robot, [:base_link, :pan_joint, :servo], 0.785)

# Bypassing pubsub, for time-critical control:
BB.Actuator.set_position!(MyRobot.Robot, :servo, 0.785)
```

The DSL takes `~u` sigil values; the runtime command functions take plain
numbers in SI base units (radians here).

- Every function accepts **either** the actuator's unique name or its full
  path — a name is resolved with `BB.Robot.actuator_path/2`. Naming an
  actuator the robot doesn't have raises rather than publishing to a topic
  nothing listens on. A full path must be complete: every link and joint from
  the root, not just the joint.
- `set_position/4` publishes via pubsub; `set_position!/4` bypasses it for
  lower latency. `set_velocity` and `set_effort` follow the same pair.
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

`use BB.Actuator`, define `init/1`, the **required** `handle_command/2`, and
the **required** `disarm/1`. Every command arrives at `handle_command/2`
whichever of the three functions above sent it — a driver can't tell, and
doesn't need to:

```elixir
alias BB.Message.Actuator.Command

def handle_command(%BB.Message{payload: %Command.Position{} = cmd}, state) do
  drive_hardware(cmd, state)
  {:noreply, state}
end

def disarm(opts), do: cut_power(opts)   # must work without GenServer state
```

Return `{:reply, reply, state}` to answer a `set_position_sync/5` caller;
`{:noreply, state}` replies `{:ok, :accepted}` for you. The reply is discarded
for the other two transports.

Don't check `BB.Safety.armed?/1` in a driver — `BB.Actuator.Server` refuses
commands to a disarmed robot before they reach you.

A command only reaches your driver if you declared it in `command_payloads/1`
**and** the robot is armed. Nothing is exempt, so a driver can't be handed a
payload it has no clause for.

`Command.Stop` is a *motion* command — cease travelling and go passive — and the
counterpart to `Command.Hold`, which maintains position and resists force. Its
`:decelerate` mode is the giveaway: nothing that slows smoothly is an emergency
stop. Making hardware safe is `disarm/1`, which is robot-wide and leaves the
robot unable to move until re-armed.

The default payload list includes `Stop`, so unless you narrow it, give it a
clause that actually stops driving rather than letting a catch-all swallow it:

```elixir
def handle_command(%BB.Message{payload: %Command.Stop{}}, state) do
  cut_drive(state)
  {:noreply, state}
end
```

`handle_info/2`, `handle_cast/2` and `handle_call/3` remain available for the
driver's own traffic (bus replies, timers, topics it subscribed to itself).
Those messages are passed through untouched.

To report position back in joint-space, either let BB publish for you
(`BB.Actuator.publish_begin_motion/3`) or translate with
`BB.Actuator.to_joint_space/3` and publish yourself.

See [Writing an Actuator](https://hexdocs.pm/bb/12-writing-an-actuator.html).
