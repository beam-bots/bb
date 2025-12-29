<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to coding assistances while working with this project.

## Project Overview

Beam Bots is a framework for building resilient robotics projects in Elixir. It provides a Spark DSL for defining robot topologies (links, joints, sensors, actuators) with automatic supervision tree generation that mirrors the physical structure for fault isolation.

## Documentation

See `documentation/tutorials/` for guided tutorials:

1. `01-first-robot.md` - defining robots with the DSL
2. `02-starting-and-stopping.md` - supervision trees and fault isolation
3. `03-sensors-and-pubsub.md` - publishing and subscribing to messages
4. `04-kinematics.md` - computing link positions with forward kinematics
5. `05-commands.md` - the command system and robot state machine
6. `06-urdf-export.md` - exporting to URDF for ROS tools
7. `07-parameters.md` - runtime-adjustable configuration
8. `08-parameter-bridges.md` - bidirectional parameter access with remote systems
9. `09-inverse-kinematics.md` - solving inverse kinematics
10. `10-simulation.md` - running robots in simulation mode

The DSL reference is in `documentation/dsls/DSL-BB.md`.

## Common Commands

```bash
# Run all checks (formatter, tests, credo, dialyzer, etc.)
mix check --no-retry

# Run tests
mix test
mix test path/to/test_file.exs           # Single file
mix test path/to/test_file.exs:42        # Single test at line

# Code quality
mix format
mix credo --strict
mix dialyzer

# Spark DSL tools
mix spark.formatter                       # Update formatter with DSL locals
mix spark.cheat_sheets                    # Generate DSL documentation

# URDF export
mix bb.to_urdf MyRobot              # Print URDF to stdout
mix bb.to_urdf MyRobot -o robot.urdf # Write to file
```

## Architecture

### Spark DSL (`lib/bb/dsl.ex`)

The core DSL defines robot structure using nested entities:
- **settings** - robot name, registry/supervisor modules
- **topology** - contains links, joints, sensors, actuators in a tree structure
- **sensors** - robot-level sensors (GPS, battery, etc.)
- **controllers** - robot-level controller processes
- **commands** - commands with handlers and state machine integration

Within the topology:
- **link** - kinematic link (solid body) with visual, collision, inertial properties
- **joint** - connection between links (revolute, prismatic, fixed, continuous, floating, planar)
- **sensor/actuator** - child processes attached to links or joints

The DSL supports physical units via `~u` sigil (e.g., `~u(0.1 meter)`, `~u(90 degree)`).

### DSL Transformers (compile-time)

Transformers run in sequence to process DSL at compile-time:
1. `DefaultNameTransformer` - sets robot name to module name if unset
2. `TopologyTransformer` - validates link hierarchy
3. `SupervisorTransformer` - generates supervision tree specs
4. `RobotTransformer` - builds optimised `BB.Robot` struct, injects `robot/0` function

### Runtime Components

**Robot struct** (`lib/bb/robot.ex`): Optimised representation with:
- Flat maps for O(1) lookup of links/joints/sensors/actuators
- All units converted to SI base (metres, radians, kg)
- Pre-computed topology for traversal

**Supervision tree** (`lib/bb/supervisor.ex`): Mirrors robot topology for fault isolation. Crashes propagate only within affected subtree.

**PubSub** (`lib/bb/pub_sub.ex`): Hierarchical message routing by path. Subscribers can match exact paths or entire subtrees.

**Kinematics** (`lib/bb/robot/kinematics.ex`): Forward kinematics using 4x4 homogeneous transform matrices (Nx tensors).

**Runtime** (`lib/bb/robot/runtime.ex`): Manages robot operational state with a state machine:
- `:disarmed` → `:idle` → `:executing` → `:idle`
- Commands only execute in allowed states
- Subscribes to sensor messages and updates joint positions

**Commands**: Defined in the DSL `commands` section with handlers implementing `BB.Command` behaviour. Built-in commands include `BB.Command.Arm` and `BB.Command.Disarm`.

**URDF Export** (`lib/bb/urdf/exporter.ex`): Converts robot definitions to URDF XML format for use with ROS tools like RViz and Gazebo. Available via `mix bb.to_urdf`.

### Simulation Mode

Robots can run in simulation mode without hardware:

```elixir
# Start in kinematic simulation
MyRobot.start_link(simulation: :kinematic)

# Check simulation mode
BB.Robot.Runtime.simulation_mode(MyRobot)  # => :kinematic or nil
```

In simulation mode:
- Actuators are replaced with `BB.Sim.Actuator` which publishes `BeginMotion` messages with timing based on joint velocity limits
- Controllers are omitted by default (configurable per-controller with `simulation: :omit | :mock | :start`)
- Safety system still requires arming before commands work
- `OpenLoopPositionEstimator` works unchanged for position feedback

See `documentation/tutorials/10-simulation.md` for details.

### Safety System (CRITICAL)

See `documentation/topics/safety.md` for comprehensive safety documentation.

**Changes to safety-critical code require extra care** - bugs here could result in physical harm.

Key points:
- Actuators controlling hardware MUST implement `BB.Safety` behaviour
- `disarm/1` callback must work without GenServer state (process may have crashed)
- Safety states: `:disarmed` → `:armed` → `:disarming` → `:disarmed` (or `:error` on failure)
- Disarm callbacks run concurrently with 5 second timeout
- The `:error` state means hardware may not be safe - requires `force_disarm/1` to recover

### Message System

`BB.Message` wraps payloads with timestamp/frame_id. Payload types use `use BB.Message` with a schema for validation via Spark.Options.

## Key Patterns

- Units: Use `Cldr.Unit` throughout DSL, converted to floats (SI) in Robot struct
- Transforms: 4x4 matrices in `BB.Math.Transform`, angles in radians
- Process registration: Uses Registry with `:via` tuples, names must be globally unique per robot
- DSL entities are structs in `lib/bb/dsl/` matching entity names
- Commands: Return `{:ok, result}` or `{:ok, result, next_state: state}` for state transitions
- State machine: Robots start `:disarmed`, transition to `:idle` when armed, `:executing` during commands
- **Safety**: Actuators controlling physical hardware must implement `BB.Safety` behaviour. Test disarm callbacks thoroughly - they run when things have already gone wrong
