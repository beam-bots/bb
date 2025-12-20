<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Safety in BeamBots

## Overview

BeamBots provides a software safety system through the `BB.Safety` module and
`BB.Safety.Controller`. This document explains how the system works and its limitations.

## How Safety Works

The safety system has four key components:

1. **Behaviour callbacks**: Actuators and controllers implement the `disarm/1` callback
   via their behaviours (`BB.Actuator`, `BB.Controller`) to handle hardware shutdown.
   Sensors can optionally implement `disarm/1` if they control hardware.
2. **Registration**: Processes register with the safety controller on startup,
   providing hardware-specific options needed for stateless disarm
3. **State management**: The controller tracks safety state per robot (`:disarmed`,
   `:armed`, or `:error`)
4. **Disarm callbacks**: On disarm command or robot crash, all registered `disarm/1`
   callbacks are invoked

### Safety States

| State | Description |
|-------|-------------|
| `:disarmed` | Robot is safely disarmed, all disarm callbacks succeeded |
| `:armed` | Robot is armed and ready to operate |
| `:disarming` | Disarm in progress, callbacks running concurrently |
| `:error` | Disarm attempted but callbacks failed; hardware may not be safe |

When disarm is called, the robot immediately transitions to `:disarming` state.
Commands are rejected while in this state. Disarm callbacks run concurrently with
a 5 second timeout per callback.

If all callbacks succeed, the robot transitions to `:disarmed`. If any callback
fails (returns an error, raises, throws, or times out), the robot transitions to
`:error` state instead. This prevents the robot from being armed again until an
operator manually acknowledges the failure using `BB.Safety.force_disarm/1`.

### Shutdown Behaviour

When the safety controller terminates (e.g. during application shutdown), it
attempts to disarm all armed robots. This is a best-effort operation - if the
system is shutting down quickly, callbacks may not complete. Always rely on
hardware safety controls for critical applications.

### Example Implementation

```elixir
defmodule MyServo do
  use GenServer
  use BB.Actuator

  @impl BB.Actuator
  def disarm(opts) do
    # This callback can be called even if the GenServer process is dead
    pin = Keyword.fetch!(opts, :pin)
    Pigpio.set_servo_pulsewidth(opts[:gpio_ref], pin, 0)
    :ok
  end

  def init(opts) do
    # Register with safety controller, providing stateless disarm options
    BB.Safety.register(__MODULE__,
      robot: opts[:bb].robot,
      path: opts[:bb].path,
      opts: [pin: opts[:pin], gpio_ref: opts[:gpio_ref]]
    )

    {:ok, %{pin: opts[:pin]}}
  end
end
```

### Key Design Decisions

- **Stateless disarm**: The `disarm/1` callback receives only the options provided
  at registration. It must work without access to GenServer state, enabling
  hardware to be made safe even when the process has crashed.

- **Direct ETS registration**: Handler registration writes directly to an ETS table,
  avoiding the controller's GenServer mailbox. This prevents registration failures
  during high load.

- **Protected state changes**: Arm/disarm state changes go through the GenServer
  to ensure callbacks are invoked and events are published.

## Important Limitations

### BEAM is Soft Real-Time

The BEAM virtual machine provides soft real-time guarantees, not hard real-time.
This means:

- **No guaranteed response times**: Disarm callbacks may be delayed by garbage
  collection, scheduler load, or other system activity
- **No guaranteed execution**: If the BEAM VM crashes, disarm callbacks won't run
- **Priority is best-effort**: While the safety controller runs at high scheduler
  priority, this doesn't guarantee immediate execution

### What This Means for Your Robot

The software safety system is suitable for:

- Hobby projects and prototypes
- Research platforms with human supervision
- Systems where delayed shutdown is acceptable

**The software safety system is NOT sufficient for:**

- Safety-critical applications
- Systems that could cause injury or property damage
- Unattended operation of potentially dangerous equipment

## Recommendations for Safety-Critical Systems

For robots where safety is critical, we strongly recommend implementing a **hardware
safety system** in addition to the software controls:

### Hardware Kill Switch

Use a dedicated microcontroller (Arduino, ESP32, STM32, etc.) with relays or
solid-state switches that can physically disconnect power to actuators:

```
┌─────────────────┐     ┌──────────────┐     ┌──────────────┐
│  BeamBots       │────▶│  Kill Switch │────▶│  Actuators   │
│  (software)     │     │  (hardware)  │     │              │
└─────────────────┘     └──────────────┘     └──────────────┘
        │                      │
        │   Heartbeat signal   │
        └──────────────────────┘
```

**Implementation options:**

1. **Watchdog heartbeat**: BeamBots sends periodic pulses. If pulses stop, hardware
   cuts power automatically
2. **Manual E-stop**: Physical button that immediately disconnects actuator power
3. **Dual-channel**: Both software command AND heartbeat required to enable actuators

### Example Hardware Setup

- ESP32 or Arduino Nano monitoring a GPIO heartbeat from the main computer
- Relay module controlling actuator power
- Physical emergency stop button
- LED indicators for system state

### Heartbeat Example

```elixir
# In your robot's controller
defmodule MyRobot.SafetyHeartbeat do
  use GenServer

  @heartbeat_interval_ms 100

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  def init(opts) do
    pin = Keyword.fetch!(opts, :heartbeat_pin)
    schedule_heartbeat()
    {:ok, %{pin: pin, gpio: opts[:gpio]}}
  end

  def handle_info(:heartbeat, state) do
    # Toggle the heartbeat pin
    Pigpio.write(state.gpio, state.pin, 1)
    Process.sleep(1)
    Pigpio.write(state.gpio, state.pin, 0)

    schedule_heartbeat()
    {:noreply, state}
  end

  defp schedule_heartbeat do
    Process.send_after(self(), :heartbeat, @heartbeat_interval_ms)
  end
end
```

## The Safety Hierarchy

When designing robot safety, think in layers:

1. **Physical E-stop** (fastest, most reliable)
   - Manual button or switch
   - Directly interrupts power
   - No software dependency

2. **Hardware watchdog** (fails safe on software crash)
   - Monitors heartbeat from BeamBots
   - Automatic power cutoff if heartbeat stops
   - Independent of BEAM VM

3. **BB.Safety controller** (software-managed, best effort)
   - Centralised arm/disarm state
   - Calls registered disarm callbacks
   - Handles robot supervisor crashes

4. **Individual process state** (application-level)
   - Per-actuator enable/disable
   - Command validation
   - Motion limits

## Summary

BeamBots' software safety system provides convenient, centralised safety management
for your robot. However, for any application where actuator failures could cause
harm, always implement hardware-level safety controls as your primary protection.
The software system should be considered a convenience layer, not a safety-critical
component.

### Quick Reference

| Question | Answer |
|----------|--------|
| Can BeamBots guarantee actuators stop within X ms? | No |
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
