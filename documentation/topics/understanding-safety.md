<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Understanding Safety in Beam Bots

This document explains the design and limitations of Beam Bots' safety system - what it does, why it works the way it does, and when you should rely on it.

## Overview

Beam Bots provides a software safety system through the `BB.Safety` module and `BB.Safety.Controller`. The system coordinates disarm operations across all hardware-controlling processes in a robot.

## Why Software Safety Exists

Physical robots need a way to quickly disable actuators when something goes wrong. Common scenarios include:

- **Process crashes** - An actuator process dies while hardware is active
- **Application shutdown** - The entire system is stopping
- **Emergency stop** - User or code triggers immediate halt
- **Error conditions** - Hardware reports faults that require shutdown

The software safety system provides a centralised way to handle all these cases with consistent behaviour.

## The Safety State Machine

Every robot has a safety state managed by `BB.Safety.Controller`:

```
:disarmed ──arm──→ :armed
    ↑                  │
    │                  │ disarm
    │                  ↓
    └───────────── :disarming
                       │
                       │ (on failure)
                       ↓
                    :error
```

| State | Description |
|-------|-------------|
| `:disarmed` | Robot is safe, all disarm callbacks succeeded |
| `:armed` | Robot is ready to operate |
| `:disarming` | Disarm in progress, callbacks running concurrently |
| `:error` | Disarm attempted but callbacks failed; hardware may not be safe |

Commands are rejected while in `:disarming` state. The `:error` state prevents re-arming until an operator acknowledges the failure with `BB.Safety.force_disarm/1`.

## Why Stateless Disarm Callbacks?

The `disarm/1` callback receives only options, not GenServer state. This design choice exists because:

1. **Process may be dead** - When a supervisor detects a crash, the process state is gone
2. **Speed matters** - Opening fresh hardware connections is slower but reliable
3. **Isolation** - Each callback can fail independently without affecting others

The trade-off: you must pass all hardware access information at registration time.

## Which Component Registers and Disarms?

Exactly one component owns each piece of hardware, and that component is
responsible for its safety: it registers with `BB.Safety.register/2` at init and
implements the `disarm/1` callback that makes its hardware safe. Other components
that share the same hardware do not register and do not disarm — they only gate
their own commands on `BB.Safety.armed?/1`.

Which component owns the hardware depends on the driver shape:

- **Standalone actuator** (independent channel — e.g. PWM servos like
  `bb_servo_pca9685`, `bb_servo_pigpio`). Each actuator owns its own GPIO pin or
  PWM channel, so **each actuator registers and implements `disarm/1`**. Its
  disarm makes that one channel safe (e.g. zero the pulse width, disable the
  channel).

- **Controller + actuator** (shared serial bus — e.g. `bb_servo_feetech`,
  `bb_servo_robotis`). A single controller owns the serial port; the actuators
  cannot open it independently. So **the controller registers and implements
  `disarm/1`**, disabling torque on every servo on the bus in one bulk write. The
  actuators delegate: their `disarm/1` is `def disarm(_opts), do: :ok` and they
  gate commands on `BB.Safety.armed?/1`.

The rule is the same in both shapes: *whoever owns the hardware connection
registers and disarms.* A no-op `disarm/1` is only safe when another component —
the controller that owns the bus — has registered to disarm that hardware. Never
leave a hardware resource with no registered owner.

See [How to Implement Safety Callbacks](../how-to/implement-safety-callbacks.md)
and [How to Integrate a Servo Driver](../how-to/integrate-servo-driver.md) for
worked examples of each shape.

## Concurrent Execution with Timeouts

Disarm callbacks run concurrently with a 5-second timeout per callback. This design reflects practical constraints:

- **Why concurrent?** - Waiting for slow hardware sequentially could take too long
- **Why 5 seconds?** - Long enough for most I2C/serial operations, short enough to be useful
- **Why timeout?** - Hung hardware shouldn't block system shutdown forever

If any callback fails, times out, or raises, the robot transitions to `:error` rather than `:disarmed`.

## Hardware Error Reporting and Escalation

Controllers and actuators can report hardware errors using `BB.Safety.report_error/3`. This publishes a `BB.Safety.HardwareError` message to `[:safety, :error]` for subscribers - it does not disarm the robot or change safety state.

Escalation is the supervisor's job. When a process detects an unrecoverable hardware fault, it should crash (raise, exit, or return `{:stop, reason, state}` from a GenServer callback). The supervision tree then decides what happens:

1. **Transient failure** - one crash, supervisor restarts the process, robot keeps running
2. **Persistent failure** - repeated crashes exhaust the subtree's restart budget, the subtree dies
3. **Topology failure** - if failures cascade up to the topology supervisor and exhaust *its* budget, the topology supervisor dies
4. **Force-disarm** - the safety controller monitors the topology supervisor; when it stops, the robot is force-disarmed and transitioned to `:error` state

This means there is no single "auto-disarm on error" switch. Instead, the restart budget on the topology supervisor (configurable via the `topology_max_restarts` and `topology_max_seconds` settings) controls how much failure the robot will tolerate before giving up.

Subscribers to `[:safety, :error]` can implement custom monitoring or alerting without affecting the escalation path.

## The Safety Hierarchy

Software safety is one layer in a multi-layered approach:

```
1. Physical E-stop (fastest, most reliable)
   ├── Manual button or switch
   ├── Directly interrupts power
   └── No software dependency

2. Hardware watchdog (fails safe on software crash)
   ├── Monitors heartbeat from Beam Bots
   ├── Automatic power cutoff if heartbeat stops
   └── Independent of BEAM VM

3. BB.Safety controller (software-managed, best effort)
   ├── Centralised arm/disarm state
   ├── Calls registered disarm callbacks
   └── Handles robot supervisor crashes

4. Individual process state (application-level)
   ├── Per-actuator enable/disable
   ├── Command validation
   └── Motion limits
```

Each layer handles failures that slip through the layer above.

## BEAM is Soft Real-Time

The BEAM virtual machine provides soft real-time guarantees, not hard real-time. This fundamental limitation shapes what the safety system can promise:

**What "soft real-time" means:**
- Processes get fair scheduling, but no guaranteed response times
- Garbage collection can pause any process
- Scheduler load affects message delivery timing
- The VM itself can crash (segfault, OOM, etc.)

**Implications for safety:**
- Disarm callbacks may be delayed by milliseconds to seconds
- If the VM crashes, no callbacks run
- High CPU load can delay safety responses

## When Software Safety is Sufficient

The software safety system is appropriate for:

- **Hobby projects and prototypes** - Where delayed shutdown is acceptable
- **Research platforms** - With human supervision during operation
- **Low-power systems** - Where uncontrolled motion causes no harm
- **Development and testing** - Before hardware safety is installed

## When to Add Hardware Safety

Add hardware-level safety for:

- **Systems that could cause injury** - Any robot near humans
- **Unattended operation** - No human to hit the stop button
- **High-power actuators** - Where runaway motion is dangerous
- **Production deployments** - Where reliability is critical

### Hardware Safety Options

**Watchdog heartbeat**: Beam Bots sends periodic pulses to a microcontroller. If pulses stop, hardware cuts power automatically. This catches VM crashes and hangs.

**Manual E-stop**: Physical button that immediately disconnects actuator power. Independent of all software.

**Dual-channel enable**: Both software command AND heartbeat required to enable actuators. Defense in depth.

## Bridging Arm/Disarm to User Commands

`BB.Safety.arm/1` and `BB.Safety.disarm/2` are the public entry points for
flipping safety state. By default they call straight into the safety
controller. If your robot needs to do work as part of arming or disarming
(e.g. moving joints to a home pose before disarming, running a self-check
before arming) you can mark a command as the canonical arm or disarm
command:

```elixir
commands do
  command :home_and_arm do
    handler MyApp.Commands.HomeAndArm
    arm true                  # ← this command IS arming
    allowed_states [:disarmed]
  end

  command :soft_disarm do
    handler MyApp.Commands.SoftDisarm
    disarm true               # ← this command IS disarming
    allowed_states [:idle]
  end
end
```

When set, `BB.Safety.arm/1` dispatches the flagged command via the runtime
(equivalent to calling `MyRobot.home_and_arm()`) and waits for its result.
The intent of "arm this robot" lives in one place regardless of whether
the caller is the safety API, an MCP client, or a Phoenix LiveView button.

Inside your command handler, call `BB.Safety.Controller.arm/1` or
`BB.Safety.Controller.disarm/2` to perform the actual state flip — these
bypass the routing layer to avoid recursion.

The built-in `BB.Command.Arm` and `BB.Command.Disarm` are implicitly
flagged, so robots using them automatically route through the command
pipeline. Default behaviour is preserved.

### Failure semantics

- **Arm command fails before flipping state** — the robot stays
  `:disarmed`. The caller receives the error.
- **Disarm command fails before flipping state** — the robot is
  escalated to `:error`. By the time a disarm sequence has been started,
  any partial completion may have left hardware in an unsafe state, so
  the operator must call `force_disarm/1` to acknowledge.
- **Disarm command flipped state then errored** — whatever state the
  controller left the robot in is preserved; the error is returned.

### DSL validation

The DSL transformer rejects:

- two commands with `arm true` (or `disarm true`),
- a single command with both flags set,
- arm-flagged commands whose `allowed_states` doesn't include `:disarmed`,
- disarm-flagged commands not reachable from any armed state.

## Shutdown Behaviour

When the safety controller terminates (e.g., during application shutdown), it attempts to disarm all armed robots. This is best-effort:

- **Normal shutdown** - Callbacks have time to complete
- **Quick shutdown** - Callbacks may be interrupted
- **VM crash** - No callbacks run

Always design physical systems assuming software may not execute cleanup.

## Quick Reference

| Question | Answer |
|----------|--------|
| Can Beam Bots guarantee actuators stop within X ms? | No |
| Is the software safety system enough for hobby projects? | Yes |
| Should I use hardware safety for research robots? | Recommended |
| Is software safety enough for unattended operation? | No |
| Can disarm callbacks run if my actuator process crashed? | Yes |
| Will disarm callbacks run if the BEAM VM crashes? | No |
| What happens if a disarm callback fails? | Robot enters `:error` state |
| Can I arm a robot in `:error` state? | No, use `force_disarm/1` first |
| Do disarm callbacks run concurrently? | Yes, with 5 second timeout |
| Can commands execute while disarming? | No, rejected with `:disarming` error |
| Are robots disarmed on shutdown? | Yes, best-effort during controller terminate |
| What happens when a hardware error is reported? | Event is published to `[:safety, :error]`; no state change. Components escalate by crashing. |
| How is the robot force-disarmed on persistent failure? | When the topology supervisor exhausts its restart budget and stops, the safety controller force-disarms and transitions to `:error`. |
| How do I tune how much failure the robot tolerates? | Set `topology_max_restarts` and `topology_max_seconds` in settings. |

## Related Documentation

- **[How to Implement Safety Callbacks](../how-to/implement-safety-callbacks.md)** - Step-by-step implementation guide
- **[How to Integrate a Servo Driver](../how-to/integrate-servo-driver.md)** - Includes safety registration patterns
