<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Simulation

Run a robot without hardware by starting it in a simulation mode:

```elixir
MyRobot.Robot.start_link(simulation: :kinematic)
BB.Robot.Runtime.simulation_mode(MyRobot.Robot)   # => :kinematic (or nil)
```

The generated `MyRobot.Application` typically wires this to the `SIMULATE`
environment variable via its `robot_opts/0`, so `SIMULATE=1 iex -S mix` boots
in simulation.

## What changes under simulation

- **Actuators** are replaced by `BB.Sim.Actuator`, which publishes
  `BeginMotion` messages timed from each joint's velocity limit.
- **Controllers default to `simulation: :omit`** — a DSL-declared controller
  does *not* start under simulation unless you set it to `:mock` or `:start`.
  This is the single most common simulation surprise.
- Open-loop position estimation still runs, so joint positions update and
  forward kinematics work unchanged.

## What does not change

**The safety system still applies.** A simulated robot starts `:disarmed` and
must be armed before it will accept motion commands — exactly like hardware.
Simulation removes the hardware, not the state machine.

See [Simulation Mode](https://hexdocs.pm/bb/10-simulation.html).
