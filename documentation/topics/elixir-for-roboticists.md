<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Elixir for Roboticists

This page is for people who already build robots — most likely in ROS, in C++ or Python — and are looking at Beam Bots wondering how the pieces map onto what they already know.

It deliberately doesn't teach Elixir. There's a good ecosystem for that already, and the [Elixir getting started guide](https://hexdocs.pm/elixir/introduction.html) is a better use of an afternoon than anything that could fit here. What this covers is the part you can't get from a language tutorial: which of your existing concepts carry over, which ones change shape, and how a finished robot actually gets onto hardware.

Going the other way — an Elixir developer who's never described a physical machine — is [Robotics Vocabulary](robotics-vocabulary.md).

## The mapping

| You know | Here it's |
|---|---|
| A node | A process. They cost about a kilobyte, so there are far more of them and you stop rationing. |
| A topic | A path in `BB.PubSub`, like `[:sensor, :base_link, :imu]`. Subscribers match a path or a whole subtree. |
| A message type | A `BB.Message` payload module with a validated schema. |
| A launch file | The robot module itself. Declaring the machine is what starts it. |
| A parameter server | `BB.Parameter` — typed, bounded, changeable at runtime, published on change. |
| URDF | The `topology` block. BB exports URDF from it, rather than reading URDF in. |
| A service call | A `BB.Command` — short-lived, has a result, and declares which states it's legal in. |
| An action | Also a `BB.Command`. It can react to messages while it runs. |
| A controller in a controller manager | A `BB.Controller`, supervised as part of the robot. |

Two rows deserve more than a table cell.

**tf, and where transforms live.** You're used to transforms being published, aggregated and looked up at runtime, with all the staleness and ordering problems that brings. In BB the transform tree is the `topology`, fixed at compile time, and a fixed joint is how you record where something is mounted. Kinematics reads it directly. That's less flexible — you can't reparent something at runtime — and in exchange there's no transform to arrive late, go stale, or be missing at startup.

**Launch files.** There isn't a separate orchestration layer that starts a set of nodes described somewhere else. The robot description *is* the process structure, and the next section is why that matters more than it sounds.

## Supervision is not a respawn loop

If a node dies, something restarts it. You have this already, whether it's a launch respawn, systemd, or a watchdog.

Supervision is a different shape, in three ways.

**It's structural.** The supervision tree mirrors the robot. The processes for a wrist sensor sit under the wrist, under the arm, under the robot. A failure is contained at the narrowest level that can deal with it, so a misbehaving sensor takes out the sensor, not the arm.

**Restarts are declared, not scripted.** A supervisor has a strategy — restart just the failed child, or restart its siblings too because they shared state with it — and a limit. Exceed the limit and the failure escalates to the supervisor above, which gets to make the same decision at a larger scale. Nobody writes retry logic.

**Crashing is a normal control-flow option.** Code here doesn't defend against every bad state. If a sensor gets a checksum failure it doesn't yet understand, letting it die and come back in a known-good state is a legitimate design, often a better one than a rescue clause guessing at recovery. This is the cultural difference that takes longest to get used to.

The payoff for a robot is that "the IMU driver wedged" stops being an incident. It's a restart, a telemetry event, and an estimator that reports degraded for a few hundred milliseconds.

## Nothing is shared

There is no shared mutable state. No mutex around a state struct, because there's no struct two things can reach.

A process owns its state and the only way to affect it is to send it a message. When you pass data to another process it's copied, and neither side can change what the other holds. That's what makes the crash-and-restart story safe: a process dying can't leave a half-updated structure behind for something else to trip over.

The practical consequence for robot code is that the question "what locks protect this?" is replaced by "which process owns this?". Usually the answer falls out of the topology — the thing that owns a sensor's state is that sensor.

## Deployment

The same robot module runs unchanged in all of the below. What varies is which driver packages you include and whether you start in simulation. Nothing about the topology, the commands or the controllers changes between a laptop and a field-deployed board.

**Nerves**, for a device that ships. You build a firmware image containing a minimal Linux, the Erlang runtime and your application, and burn or push it to the board. There's no distribution underneath — no package manager, no init system to configure, a read-only root filesystem, and A/B partitions so a failed update rolls back instead of bricking the thing. This is the right answer for anything deployed somewhere inconvenient. The cost is that you need a Nerves system for your board, you're cross-compiling, and any native dependency has to build for the target. See [Deploy to Nerves](../how-to/deploy-to-nerves.md).

Once there's more than one robot, the piece that makes this tolerable is [NervesHub](https://nerves-hub.org). It's an open-source device management server — itself an Elixir application — that handles staged over-the-air firmware rollouts, collects health metrics off the fleet, alerts you when a device starts misbehaving, and will drop you into an IEx console on a specific device wherever it happens to be. That last one is worth dwelling on if you've ever tried to debug a robot in someone else's building: it's the same live-inspection story as the [toolchain](#toolchain) section below, against hardware on another continent. Devices connect with the [`nerves_hub_link`](https://hex.pm/packages/nerves_hub_link) library.

You can self-host it. If you'd rather not run the server, [NervesCloud](https://nervescloud.com) is the hosted version of the same thing, from the people who maintain it.

**`mix release`, onto a Linux board you manage.** A release is a self-contained directory with the runtime bundled, which you drop on a Debian or Ubuntu SBC and run under systemd. Pick this when you need a real distribution underneath — vendor SDKs, GPU stacks, an existing ROS install to bridge to — or when your board has no Nerves system. You keep the OS, and the patching of it.

**A container.** `mix release` inside an image, run under Docker, podman or Kubernetes. This is the natural fit for the parts of a system that aren't holding a motor: simulation, CI, fleet-facing services, dashboards. On the robot itself it's more awkward than it looks, because the hardware libraries need the actual device nodes — `/dev/i2c-*`, `/dev/spidev*`, the GPIO character devices — passed through, and handing a container enough privilege to do that gives away a lot of what the container was for.

**Nothing at all.** In simulation the robot runs on your laptop with no hardware present, actuators replaced by simulated ones and controllers omitted by default. This is a first-class mode rather than a testing hack, and it's how most development happens. See [Simulation](../tutorials/10-simulation.md).

The thing that usually decides between the first two is not the software. It's whether you can get at the peripherals you need, and whether anyone will be able to reach the device to fix it.

## Toolchain

Nothing needs sourcing before you can use it. `mix` is the build tool, the test runner and the task runner; `hex` is the package registry; dependencies are versioned in `mix.exs`, locked in `mix.lock`, and vendored into the project rather than installed system-wide. There's no separate build, install and overlay step, and no workspace to configure.

Two habits transfer badly:

- **There's no system package layer for Elixir libraries.** Everything comes from Hex and lives in the project. What you do still need the OS for is C libraries a native dependency links against, which on Nerves means the system definition rather than `apt`.
- **A running robot is inspectable.** You connect to the target with IEx and call functions on the live system — read a parameter, send a command, inspect a process's state. It replaces a surprising amount of what you'd otherwise reach for a debugger or a bag file to do.

## Where to go next

- [Robotics Vocabulary](robotics-vocabulary.md) — the same glossary from the other side, and useful for the BB-specific terms
- [Your First Robot](../tutorials/01-first-robot.md) — start here once the vocabulary lines up
- [Starting and Stopping](../tutorials/02-starting-and-stopping.md) — the supervision tree in practice
- [Understanding Safety](understanding-safety.md) — arming, disarming, and what happens when a component can't be made safe
- [Use URDF with ROS Tools](../how-to/use-urdf-with-ros.md) — exporting to RViz and friends
