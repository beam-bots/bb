# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Kinetix is a framework for building resilient robotics projects in Elixir. It's currently in early development (v0.1.0) on the `dsl-spike` branch, focusing on a Spark DSL for describing robot topologies.

The project uses Spark (from the Ash Framework ecosystem) to provide an extensible DSL for defining robot kinematics, similar to URDF but as Elixir code.

## Architecture

### Core Abstractions

**Spark DSL System**: Kinetix is built on Spark's DSL extension system, providing two main extensions:

- `Kinetix.Base` - Defines the `robot` section for universal robot properties (name, etc)
- `Kinetix.Topology` - Defines the `topology` section for describing kinematic chains

**Entities**: The topology DSL defines several entity types:

- `Link` - Represents a solid body in the kinematic chain (lib/kinetix/topology/link.ex)
- `Joint` - Connects two links with motion constraints (lib/kinetix/topology/joint.ex)
  - Types: `:revolute`, `:continuous`, `:prismatic`, `:fixed`, `:floating`, `:planar`
  - Contains nested entities: `origin`, `axis`, `calibration`, `dynamics`
- `Origin` - Transform from parent to child link (translation + rotation)
- `Axis` - Joint axis for revolute/prismatic/planar joints
- `Calibration` - Reference positions for joint calibration (rising/falling edges)
- `Dynamics` - Physical properties (damping, friction) for simulation

### Unit System

`Kinetix.Unit` (lib/kinetix/unit.ex) provides:

- Helper functions for all CLDR units (e.g., `meter(5)`, `degree(90)`)
- `schema_type/1` for creating validated Spark schema types with constraints:
  - `category:` - Restrict to a unit category (`:length`, `:angle`, etc)
  - `min:`, `max:`, `eq:` - Numerical constraints
- Automatic unit validation ensuring consistent categories across constraints

Units are powered by `ex_cldr_units` with locale configuration in `Kinetix.Cldr`.

### Module Structure

```
lib/kinetix.ex              # Main DSL entry point (use Kinetix)
lib/kinetix/base.ex         # Base DSL extension (robot section)
lib/kinetix/topology.ex     # Topology DSL extension (links/joints)
lib/kinetix/topology/       # Entity struct definitions
lib/kinetix/unit.ex         # Unit helpers and validation
lib/kinetix/cldr.ex         # CLDR configuration
```

### DSL Usage Pattern

Modules using Kinetix define robot topologies like this:

```elixir
defmodule MyRobot do
  use Kinetix

  robot do
    name :my_robot

    topology do
      link :base_link

      joint :joint1 do
        type :revolute
        parent :base_link
        child :link1

        origin do
          x meter(0.1)
          yaw degree(45)
        end

        axis do
          z meter(1)
        end
      end
    end
  end
end
```

## Development Commands

### Running Tests

```bash
# All tests
mix test

# Single test file
mix test test/path/to/file_test.exs

# Single test by line number
mix test test/path/to/file_test.exs:42

# Watch mode (if available)
mix test.watch
```

### Quality Checks

The project uses `ex_check` to coordinate quality tools:

```bash
# Run all checks (credo, dialyzer, tests, etc)
mix check --no-retry
```

Individual tools:

```bash
# Type checking
mix dialyzer

# Linting
mix credo

# Format code
mix format

# Security audit
mix deps.audit
```

### Documentation

```bash
# Generate and open docs
mix docs

# View in browser (after generation)
open doc/index.html
```

### Dependencies

```bash
# Install dependencies
mix deps.get

# Update dependencies
mix deps.update --all

# Check for outdated deps
mix hex.outdated
```

## Development Environment

- Erlang 28.2
- Elixir 1.19.4 (managed via `.tool-versions` for asdf)

## Key Dependencies

- `spark` (~> 2.3) - DSL framework (from Ash ecosystem)
- `ex_cldr_units` (~> 3.0) - Unit conversions and formatting
- `ex_cldr_numbers` (~> 2.36) - Number localisation
- `igniter` (~> 0.6) - Code generation and project patching (dev/test)

## Current Status

This is an early spike on the DSL design. The following are planned but not yet implemented:

- `Kinetix.Message` - Protocol for message serialisation
- `Kinetix.PubSub` - Cluster-aware pubsub for kinetic messages
- Additional packages: `kinetix_sitl`, `kinetix_rc_servo`, `kinetix_mavlink`, `kinetix_csrf`

See README.md for the full roadmap.

## Spark DSL Conventions

When working with Spark DSL extensions:

- Use `Entity` structs to define DSL entities with schema validation
- Use `Section` structs to group related entities
- Entity identifiers can be `:name` (user-provided) or `{:auto, :unique_integer}`
- Use `singleton_entity_keys` for entities that can only appear once (like `:origin`, `:axis`)
- Import helper modules (like `Kinetix.Unit`) at entity level for use in nested blocks
- Entity structs have `__identifier__` and `__spark_metadata__` fields automatically added

## Testing Patterns

Tests use ExUnit with `async: true` for parallel execution where possible.

Test modules for DSL functionality define inline modules using the DSL (see test/kinematix/topology_test.exs for examples).
