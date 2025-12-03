# AGENTS.md

This file provides guidance to coding assistances while working with this project.

## Project Overview

Kinetix is a framework for building resilient robotics projects in Elixir. It provides a Spark DSL for defining robot topologies (links, joints, sensors, actuators) with automatic supervision tree generation that mirrors the physical structure for fault isolation.

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
```

## Architecture

### Spark DSL (`lib/kinetix/dsl.ex`)

The core DSL defines robot structure using nested entities:
- **robot** (top-level section) - contains settings, links, joints, sensors
- **link** - kinematic link (solid body) with visual, collision, inertial properties
- **joint** - connection between links (revolute, prismatic, fixed, etc.)
- **sensor/actuator** - child processes attached to links or joints

The DSL supports physical units via `~u` sigil (e.g., `~u(0.1 meter)`, `~u(90 degree)`).

### DSL Transformers (compile-time)

Transformers run in sequence to process DSL at compile-time:
1. `DefaultNameTransformer` - sets robot name to module name if unset
2. `LinkTransformer` - validates link hierarchy
3. `SupervisorTransformer` - generates supervision tree specs
4. `RobotTransformer` - builds optimised `Kinetix.Robot` struct, injects `robot/0` function

### Runtime Components

**Robot struct** (`lib/kinetix/robot.ex`): Optimised representation with:
- Flat maps for O(1) lookup of links/joints/sensors/actuators
- All units converted to SI base (meters, radians, kg)
- Pre-computed topology for traversal

**Supervision tree** (`lib/kinetix/supervisor.ex`): Mirrors robot topology for fault isolation. Crashes propagate only within affected subtree.

**PubSub** (`lib/kinetix/pub_sub.ex`): Hierarchical message routing by path. Subscribers can match exact paths or entire subtrees.

**Kinematics** (`lib/kinetix/robot/kinematics.ex`): Forward kinematics using 4x4 homogeneous transform matrices (Nx tensors).

### Message System

`Kinetix.Message` wraps payloads with timestamp/frame_id. Payload types implement a behaviour and protocol for schema validation via Spark.Options.

## Key Patterns

- Units: Use `Cldr.Unit` throughout DSL, converted to floats (SI) in Robot struct
- Transforms: 4x4 matrices in `Kinetix.Robot.Transform`, angles in radians
- Process registration: Uses Registry with `:via` tuples, names must be globally unique per robot
- DSL entities are structs in `lib/kinetix/dsl/` matching entity names
