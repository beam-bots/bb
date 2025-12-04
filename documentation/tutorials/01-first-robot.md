<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Your First Robot

This tutorial guides you through defining your first robot with Kinetix. By the end, you'll understand the core DSL concepts and have a working robot definition.

## Prerequisites

- Elixir 1.19 or later
- Kinetix installed in your project

### Quick Start with Igniter

The fastest way to get started is with [Igniter](https://hex.pm/packages/igniter):

```bash
mix igniter.install kinetix
```

This creates a `{YourApp}.Robot` module with arm/disarm commands and a base link, adds it to your supervision tree, and configures the formatter. You can skip to [Step 2](#step-2-add-a-joint-and-child-link) and modify the generated module.

### Manual Installation

If you prefer to create the module manually, add Kinetix to your dependencies:

```elixir
# mix.exs
def deps do
  [
    {:kinetix, "~> 0.1"}
  ]
end
```

## What We're Building

We'll create a simple two-link robot arm: a base that can rotate (pan), with an arm that can tilt up and down. This is similar to a pan-tilt camera mount.

```
    [camera]     <- tilt joint rotates this
        |
    [pan_link]   <- pan joint rotates this
        |
    [base]       <- fixed to the world
```

## Step 1: Create the Module

Create a new file `lib/my_robot.ex`:

```elixir
defmodule MyRobot do
  use Kinetix

  topology do
    link :base do
    end
  end
end
```

Let's break this down:

- `use Kinetix` brings in the Kinetix DSL and the `~u` sigil for physical units
- `topology do ... end` defines the robot's physical structure
- `link :base do ... end` creates our first link (rigid body)

Compile and test:

```elixir
iex> MyRobot.robot()
%Kinetix.Robot{name: MyRobot, links: %{base: %Kinetix.Robot.Link{...}}, ...}
```

The `robot/0` function returns a compiled struct optimised for runtime use.

## Step 2: Add a Joint and Child Link

Joints connect links. Let's add a pan joint that allows rotation around the Z-axis:

```elixir
defmodule MyRobot do
  use Kinetix

  topology do
    link :base do
      joint :pan_joint do
        type(:revolute)

        axis do
        end

        link :pan_link do
        end
      end
    end
  end
end
```

Key concepts:

- **Joints are nested inside links** - the parent link contains the joint definition
- **type(:revolute)** - a revolute joint rotates around an axis (like a hinge)
- **axis** - defines which axis the joint rotates around. An empty axis block defaults to the Z-axis. You can specify different orientations using `roll`, `pitch`, and `yaw`.
- **Child link is nested inside the joint** - this creates the kinematic chain

> **For Roboticists:** The DSL compiles to an Elixir struct at compile-time. There's no runtime parsing - the robot definition is baked into your module.

> **For Elixirists:** Links are rigid bodies (solid pieces). Joints are the connections between them that allow movement. A revolute joint is like a door hinge - it rotates around one axis.

## Step 3: Add Joint Limits

Real joints have physical constraints. Let's limit the pan joint's range of motion:

```elixir
joint :pan_joint do
  type(:revolute)

  axis do
  end

  limit do
    lower(~u(-90 degree))
    upper(~u(90 degree))
    effort(~u(5 newton_meter))
    velocity(~u(60 degree_per_second))
  end

  link :pan_link do
  end
end
```

The `~u()` sigil creates unit-aware values:

- `~u(-90 degree)` - negative 90 degrees
- `~u(5 newton_meter)` - maximum torque the joint can apply
- `~u(60 degree_per_second)` - maximum rotation speed

These units are automatically converted to SI base units (radians, newton-metres) in the compiled robot struct.

## Step 4: Position the Joint

By default, joints are at the origin of their parent link. Use `origin` to offset them:

```elixir
joint :pan_joint do
  type(:revolute)

  origin do
    z(~u(0.05 meter))
  end

  axis do
  end

  limit do
    lower(~u(-90 degree))
    upper(~u(90 degree))
    effort(~u(5 newton_meter))
    velocity(~u(60 degree_per_second))
  end

  link :pan_link do
  end
end
```

The joint is now 5cm above the base link's origin.

## Step 5: Add a Second Joint

Let's add a tilt joint to create a full pan-tilt mechanism:

```elixir
defmodule MyRobot do
  use Kinetix

  topology do
    link :base do
      joint :pan_joint do
        type(:revolute)

        origin do
          z(~u(0.05 meter))
        end

        axis do
        end

        limit do
          lower(~u(-90 degree))
          upper(~u(90 degree))
          effort(~u(5 newton_meter))
          velocity(~u(60 degree_per_second))
        end

        link :pan_link do
          joint :tilt_joint do
            type(:revolute)

            origin do
              z(~u(0.03 meter))
            end

            axis do
              roll(~u(-90 degree))
            end

            limit do
              lower(~u(-45 degree))
              upper(~u(90 degree))
              effort(~u(2 newton_meter))
              velocity(~u(45 degree_per_second))
            end

            link :camera_link do
            end
          end
        end
      end
    end
  end
end
```

The tilt joint rotates around the Y-axis (specified by `roll(~u(-90 degree))` which rotates the default Z-axis to point along Y), allowing the camera to look up and down.

## Step 6: Add Visual Geometry

To visualise the robot, add visual geometry to each link:

```elixir
link :base do
  visual do
    cylinder do
      radius(~u(0.04 meter))
      height(~u(0.05 meter))
    end

    material do
      color do
        red(0.2)
        green(0.2)
        blue(0.2)
        alpha(1.0)
      end
    end
  end

  joint :pan_joint do
    # ... joint definition
  end
end
```

Available geometry types:

- `box` - with `x`, `y`, `z` dimensions
- `cylinder` - with `radius` and `height`
- `sphere` - with `radius`
- `mesh` - with `filename` for custom 3D models

## Complete Example

Here's the full robot definition:

```elixir
defmodule MyRobot do
  use Kinetix

  topology do
    link :base do
      visual do
        cylinder do
          radius(~u(0.04 meter))
          height(~u(0.05 meter))
        end

        material do
          color do
            red(0.2)
            green(0.2)
            blue(0.2)
            alpha(1.0)
          end
        end
      end

      joint :pan_joint do
        type(:revolute)

        origin do
          z(~u(0.05 meter))
        end

        axis do
        end

        limit do
          lower(~u(-90 degree))
          upper(~u(90 degree))
          effort(~u(5 newton_meter))
          velocity(~u(60 degree_per_second))
        end

        link :pan_link do
          visual do
            origin do
              z(~u(0.015 meter))
            end

            box do
              x(~u(0.03 meter))
              y(~u(0.03 meter))
              z(~u(0.03 meter))
            end

            material do
              color do
                red(0.3)
                green(0.3)
                blue(0.3)
                alpha(1.0)
              end
            end
          end

          joint :tilt_joint do
            type(:revolute)

            origin do
              z(~u(0.03 meter))
            end

            axis do
              roll(~u(-90 degree))
            end

            limit do
              lower(~u(-45 degree))
              upper(~u(90 degree))
              effort(~u(2 newton_meter))
              velocity(~u(45 degree_per_second))
            end

            link :camera_link do
              visual do
                box do
                  x(~u(0.05 meter))
                  y(~u(0.03 meter))
                  z(~u(0.03 meter))
                end

                material do
                  color do
                    red(0.1)
                    green(0.1)
                    blue(0.1)
                    alpha(1.0)
                  end
                end
              end
            end
          end
        end
      end
    end
  end
end
```

## Exploring the Compiled Robot

The `robot/0` function returns a `Kinetix.Robot` struct:

```elixir
iex> robot = MyRobot.robot()
iex> Map.keys(robot.links)
[:base, :pan_link, :camera_link]

iex> Map.keys(robot.joints)
[:pan_joint, :tilt_joint]

iex> robot.joints.pan_joint.type
:revolute

iex> robot.joints.pan_joint.limit.upper
1.5707963267948966  # 90 degrees in radians
```

Notice that angles are stored in radians (SI units) even though we defined them in degrees.

## Joint Types

Kinetix supports six joint types:

| Type | Description | Use Case |
|------|-------------|----------|
| `:revolute` | Rotation with limits | Arm joints, pan-tilt |
| `:continuous` | Unlimited rotation | Wheels |
| `:prismatic` | Linear sliding | Linear actuators |
| `:fixed` | No movement | Welded connections |
| `:floating` | 6 degrees of freedom | Free-floating objects |
| `:planar` | Movement in a plane | Some mobile bases |

## What's Next?

You've defined a robot structure, but it's not running yet. In the next tutorial, we'll:

- Start the robot's supervision tree
- Understand how the process structure mirrors the physical structure
- Learn about Kinetix's fault isolation model

Continue to [Starting and Stopping](02-starting-and-stopping.md).
