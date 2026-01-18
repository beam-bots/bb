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

## Concurrent Execution with Timeouts

Disarm callbacks run concurrently with a 5-second timeout per callback. This design reflects practical constraints:

- **Why concurrent?** - Waiting for slow hardware sequentially could take too long
- **Why 5 seconds?** - Long enough for most I2C/serial operations, short enough to be useful
- **Why timeout?** - Hung hardware shouldn't block system shutdown forever

If any callback fails, times out, or raises, the robot transitions to `:error` rather than `:disarmed`.

## Hardware Error Reporting

Controllers and actuators can report hardware errors using `BB.Safety.report_error/3`. The default behaviour is automatic disarm, which is safe but conservative.

When to disable auto-disarm:
- **Transient errors** - Brief communication glitches that self-resolve
- **Partial failures** - One servo overheating shouldn't stop the whole robot
- **Custom recovery** - You want to try recovery before disarming

Disabling auto-disarm requires implementing your own error handling via PubSub subscription to `[:safety, :error]`.

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
| What happens when a hardware error is reported? | Auto-disarm (default) or custom handling |
| How do I disable auto-disarm on error? | Set `auto_disarm_on_error false` in settings |

## Related Documentation

- **[How to Implement Safety Callbacks](../how-to/implement-safety-callbacks.md)** - Step-by-step implementation guide
- **[How to Integrate a Servo Driver](../how-to/integrate-servo-driver.md)** - Includes safety registration patterns
