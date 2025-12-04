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
- **Command system** - state machine with arm/disarm and custom commands
- **URDF export** - export robot definitions for use with ROS tools

## Example

```elixir
defmodule MyRobot do
  use Kinetix

  topology do
    link :base do
      joint :shoulder do
        type(:revolute)

        origin do
          z(~u(0.1 meter))
        end

        axis do
        end

        limit do
          effort(~u(10 newton_meter))
          velocity(~u(1 radian_per_second))
        end

        link :upper_arm do
          joint :elbow do
            type(:revolute)

            origin do
              z(~u(0.3 meter))
            end

            axis do
              roll(~u(-90 degree))
            end

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(1 radian_per_second))
            end

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
positions = %{shoulder: :math.pi() / 4, elbow: 0.0}
{x, y, z} = Kinetix.Robot.Kinematics.link_position(robot, positions, :forearm)

# Export to URDF
mix kinetix.to_urdf MyRobot -o robot.urdf
```

## Documentation

See the [tutorials](documentation/tutorials/) for a guided introduction:

1. [Your First Robot](documentation/tutorials/01-first-robot.md) - defining robots with the DSL
2. [Starting and Stopping](documentation/tutorials/02-starting-and-stopping.md) - supervision trees
3. [Sensors and PubSub](documentation/tutorials/03-sensors-and-pubsub.md) - publishing and subscribing to messages
4. [Forward Kinematics](documentation/tutorials/04-kinematics.md) - computing link positions
5. [Commands and State Machine](documentation/tutorials/05-commands.md) - controlling the robot
6. [Exporting to URDF](documentation/tutorials/06-urdf-export.md) - interoperability with ROS tools

The [DSL Reference](documentation/dsls/DSL-Kinetix.md) documents all available options.

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

