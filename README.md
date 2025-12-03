<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Kinetix

[![Build Status](https://drone.harton.dev/api/badges/james/kinetix/status.svg)](https://drone.harton.dev/james/kinetix)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/kinetix.svg)](https://hex.pm/packages/kinetix)
[![REUSE status](https://api.reuse.software/badge/harton.dev/james/kinetix)](https://api.reuse.software/info/harton.dev/james/kinetix)

Kinetix is a framework for building resilient robotics projects in Elixir.

## Features

- **Spark DSL** for defining robot topologies (links, joints, sensors, actuators)
- **Physical units** via `~u` sigil with automatic SI conversion (e.g., `~u(90 degree)`, `~u(0.1 meter)`)
- **Topology-based supervision** - supervision tree mirrors robot structure for fault isolation
- **Hierarchical PubSub** - subscribe to messages by path or subtree
- **Forward kinematics** - compute link positions using Nx tensors
- **Message system** - typed payloads with schema validation

## Example

```elixir
defmodule MyRobot do
  use Kinetix
  import Kinetix.Unit

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        origin x: ~u(0 meter), y: ~u(0 meter), z: ~u(0.1 meter)
        axis z: ~u(1 meter)
        limit effort: ~u(10 newton_meter), velocity: ~u(1 radian_per_second)

        link :upper_arm do
          joint :elbow, type: :revolute do
            origin z: ~u(0.3 meter)
            axis y: ~u(1 meter)
            limit effort: ~u(10 newton_meter), velocity: ~u(1 radian_per_second)

            link :forearm do
            end
          end
        end
      end
    end
  end
end

# Start the supervision tree
{:ok, _pid} = Kinetix.Supervisor.start_link(MyRobot)

# Compute forward kinematics
robot = MyRobot.robot()
{:ok, state} = Kinetix.Robot.State.new(robot)
Kinetix.Robot.State.set_joint_position(state, :shoulder, :math.pi() / 4)
{x, y, z} = Kinetix.Robot.Kinematics.link_position(robot, state, :forearm)
```

## Status

Core functionality is implemented. Planned additions:

- `kinetix_sitl` - simulation integration (Gazebo, etc.)
- `kinetix_rc_servo` - PWM-based RC servo driver
- `kinetix_mavlink` - MAVLink protocol bridge
- `kinetix_crossfire` - Crossfire RC bridge

## Installation

Kinetix is not yet available on Hex. Add it as a Git dependency:

```elixir
def deps do
  [
    {:kinetix, git: "https://harton.dev/kinetix/kinetix.git"}
  ]
end
```

