<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Safety and Commands

A robot has an operational state machine:
`:disarmed → :idle → :executing → :idle`. It starts `:disarmed` and **will not
move** until armed. Arming is not a flag you set — it runs the prearm checks a
robot may define and, for hardware components, the reverse (`disarm`) path.

## Arming and disarming

Arm and disarm are ordinary commands. Declare them in the DSL:

```elixir
commands do
  command :arm do
    handler BB.Command.Arm
    allowed_states [:disarmed]
  end

  command :disarm do
    handler BB.Command.Disarm
    allowed_states [:idle]
  end
end
```

Each declared command becomes a function on the robot module. Commands are
short-lived processes; call the generated function to start one, then
`BB.Command.await/2` for the result:

```elixir
{:ok, cmd} = MyRobot.Robot.arm()
{:ok, :armed, _opts} = BB.Command.await(cmd)
```

- **Never call `BB.Safety` (or its internal controller) directly to change
  state.** Going through the command system is what runs the user's prearm
  checks. Poking safety directly bypasses them.
- `allowed_states` gates when a command may run. A command invoked from a
  disallowed state fails with a `BB.Error.State` error rather than executing.

## Writing a command

`use BB.Command`, implement `handle_command/3` and `result/1`. Return
`{:ok, result}`, or `{:ok, result, next_state: state}` to drive the state
machine (this is how `Arm`/`Disarm` transition it):

```elixir
defmodule MyRobot.Command.Home do
  use BB.Command

  def handle_command(_goal, context, state) do
    # context carries the compiled robot struct; move via BB.Motion / BB.Actuator
    {:ok, state}
  end

  def result(state), do: {:ok, :home}
end
```

A command can also react to messages mid-execution and to safety changes via
optional callbacks (`handle_info/2`, `handle_safety_state_change/2`).

## The disarm contract (hardware components)

Components that drive hardware implement a `disarm/1` callback from their
behaviour — required for `BB.Actuator`, optional for `BB.Controller` and
`BB.Sensor` — and register with `BB.Safety.register/2`. `disarm/1` receives the
opts given at registration and **must work without GenServer state**: it runs
when things have already gone wrong, possibly after the process has crashed.
Disarm callbacks run concurrently with a 5-second timeout; a failure moves the
robot to `:error`, which needs `BB.Safety.force_disarm/1` to clear.

See [Commands and State Machine](https://hexdocs.pm/bb/05-commands.html) and
[Understanding Safety](https://hexdocs.pm/bb/understanding-safety.html).
