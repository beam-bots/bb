<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots

[![CI](https://github.com/beam-bots/bb/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb.svg)](https://hex.pm/packages/bb)
[![Hexdocs badge](https://img.shields.io/badge/docs-hexdocs-purple)](https://hexdocs.pm/bb)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb)](https://api.reuse.software/info/github.com/beam-bots/bb)
[![OpenSSF Best Practices](https://www.bestpractices.dev/projects/11772/badge)](https://www.bestpractices.dev/projects/11772)
[![OpenSSF Scorecard](https://api.scorecard.dev/projects/github.com/beam-bots/bb/badge)](https://scorecard.dev/viewer/?uri=github.com/beam-bots/bb)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/beam-bots/bb)

Beam Bots is a framework for building resilient robotics projects in Elixir.

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
  use BB

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
{:ok, _pid} = BB.Supervisor.start_link(MyRobot)

# Compute forward kinematics
robot = MyRobot.robot()
positions = %{shoulder: :math.pi() / 4, elbow: 0.0}
{x, y, z} = BB.Robot.Kinematics.link_position(robot, positions, :forearm)

# Export to URDF
mix bb.to_urdf MyRobot -o robot.urdf
```

## Documentation

See the tutorials for a guided introduction:

1. [Your First Robot](https://hexdocs.pm/bb/01-first-robot.html) - defining robots with the DSL
2. [Starting and Stopping](https://hexdocs.pm/bb/02-starting-and-stopping.html) - supervision trees
3. [Sensors and PubSub](https://hexdocs.pm/bb/03-sensors-and-pubsub.html) - publishing and subscribing to messages
4. [Forward Kinematics](https://hexdocs.pm/bb/04-kinematics.html) - computing link positions
5. [Commands and State Machine](https://hexdocs.pm/bb/05-commands.html) - controlling the robot
6. [Exporting to URDF](https://hexdocs.pm/bb/06-urdf-export.html) - interoperability with ROS tools
7. [Parameters](https://hexdocs.pm/bb/07-parameters.html) - runtime-adjustable configuration
8. [Parameter Bridges](https://hexdocs.pm/bb/08-parameter-bridges.html) - bidirectional remote access

The [DSL Reference](https://hexdocs.pm/bb/dsl-bb.html) documents all available options.

## Status

Core functionality is implemented. Companion packages:

- [`bb_kino`](https://github.com/beam-bots/bb_kino) - Livebook widgets for robot control and visualisation
- [`bb_liveview`](https://github.com/beam-bots/bb_liveview) - Phoenix LiveView dashboard
- [`bb_ik_fabrik`](https://github.com/beam-bots/bb_ik_fabrik) - FABRIK inverse kinematics solver
- [`bb_servo_pca9685`](https://github.com/beam-bots/bb_servo_pca9685) - PCA9685 PWM servo driver (I2C, 16-channel)
- [`bb_servo_pigpio`](https://github.com/beam-bots/bb_servo_pigpio) - pigpio servo driver (Raspberry Pi GPIO)
- [`bb_servo_robotis`](https://github.com/beam-bots/bb_servo_robotis) - Robotis/Dynamixel servo driver

See [proposals](https://github.com/beam-bots/proposals) for planned features.

## Installation

### With Igniter (Recommended)

If your project uses [Igniter](https://hex.pm/packages/igniter):

```bash
mix igniter.install bb
```

This will:
- Add Beam Bots to your dependencies
- Create a `{YourApp}.Robot` module with arm/disarm commands and a base link
- Add the robot to your application supervision tree
- Configure the formatter for the Beam Bots DSL

To add additional robots later:

```bash
mix bb.add_robot --robot MyApp.Robots.SecondRobot
```

### Manual Installation

Add Beam Bots to your dependencies:

```elixir
def deps do
  [
    {:bb, "~> 0.1"}
  ]
end
```

Then create a robot module manually (see [Your First Robot](https://hexdocs.pm/bb/01-first-robot.html)).

## Sponsors

This project is made possible by the generous support of our sponsors:

- **[Alembic](https://alembic.com.au)** ([@team-alembic](https://github.com/team-alembic)) - Development Support
- **Frank Hunleth** ([@fhunleth](https://github.com/fhunleth)) - Hardware Donation
- **Pascal Charbonneau** ([@pcharbon70](https://github.com/pcharbon70)) - GitHub Sponsor
