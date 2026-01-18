<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Understanding the Supervision Architecture

This document explains why Beam Bots generates supervision trees that mirror physical robot topology, and what benefits this architecture provides.

## The Core Idea

A robot's supervision tree structure mirrors its physical structure. When you define a robot with links and joints, the generated supervision tree has the same hierarchy:

```
Physical Structure          Supervision Tree
================           =================
Base                       BaseSupervisor
  └── Shoulder Joint         └── ShoulderSupervisor
      └── Upper Arm              ├── ShoulderActuator
          └── Elbow Joint        ├── ShoulderSensor
              └── Forearm        └── UpperArmSupervisor
                                     └── ElbowSupervisor
                                         ├── ElbowActuator
                                         └── ElbowSensor
```

This isn't accidental - it's a deliberate design choice with significant implications.

## Why Mirror Physical Structure?

### Fault Isolation

Physical robots have natural failure boundaries. If an elbow servo fails, it shouldn't affect the shoulder. The supervision tree enforces this:

- Crashes propagate only within affected subtrees
- Unaffected parts of the robot continue operating
- Recovery attempts are localised to the failed component

Consider what happens when an elbow actuator crashes:

```
                    RobotSupervisor
                          │
                    BaseSupervisor
                          │
                  ShoulderSupervisor
                    /           \
         ShoulderActuator    UpperArmSupervisor
         ShoulderSensor            │
                            ElbowSupervisor
                              /         \
                    [ElbowActuator]   ElbowSensor
                         ↑
                     (crash!)
```

The crash stays within `ElbowSupervisor`. Shoulder components keep running. The robot can potentially continue operating with reduced capability.

### Restart Strategies

Each supervisor can have its own restart strategy. BB uses `:one_for_one` by default, meaning sibling processes restart independently. But the hierarchy means:

- If an actuator keeps crashing, eventually its supervisor restarts
- When a supervisor restarts, all its children restart
- This cascades up only as far as necessary

A problematic elbow doesn't restart the entire robot - just the elbow subtree.

### Resource Management

Physical components often share resources within their kinematic chain:

- Controllers managing multiple servos on one bus
- Sensors reading from the same joint
- Coordination between actuator and sensor for position feedback

The supervision tree keeps related processes close together, supervised by the same parent.

## How the Tree is Generated

The `BB.Supervisor.SupervisorTransformer` processes the DSL at compile time:

1. Walks the topology tree (links, joints)
2. Collects sensors, actuators, and controllers at each level
3. Generates supervisor specifications that match the structure
4. Stores the spec in the compiled robot struct

When you call `BB.Supervisor.start_link/2`, it reads the pre-generated spec and starts the tree.

### Robot-Level vs Topology-Level Processes

Some processes belong to the robot as a whole, not specific links:

```elixir
defmodule MyRobot do
  use BB

  # Robot-level controller (manages I2C bus)
  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1"}
  end

  # Robot-level sensor (battery monitor)
  sensors do
    sensor :battery, {BatteryMonitor, pin: 0}
  end

  topology do
    # Joint-level processes
    link :base do
      joint :shoulder do
        actuator :servo, {...}
        sensor :position, {...}
      end
    end
  end
end
```

Robot-level processes are supervised directly under the main robot supervisor, parallel to the topology subtree.

## The Runtime Process

`BB.Robot.Runtime` is special - it's the coordinator for the entire robot:

- Manages operational state (disarmed, idle, executing)
- Subscribes to sensor messages and updates joint positions
- Spawns and monitors command processes
- Lives at the root level, sibling to the topology

```
RobotSupervisor
├── Runtime
├── SafetyController  (if safety enabled)
├── PCA9685Controller (robot-level controller)
├── BatteryMonitor    (robot-level sensor)
└── TopologySupervisor
    └── ...
```

If the Runtime crashes, it doesn't take down the topology. Hardware processes keep running while Runtime restarts and resubscribes.

## Process Registration

Every process in the tree registers with a unique name based on its path:

- `[:joint, :shoulder, :servo]` - shoulder servo actuator
- `[:joint, :elbow, :position]` - elbow position sensor
- `[:controller, :pca9685]` - robot-level controller

This enables:
- Looking up any process by path
- Addressing messages to specific components
- Debugging which process is which

Registration uses Elixir's Registry with the robot module as the key namespace.

## Starting with Options

`BB.Supervisor.start_link/2` accepts options that affect the tree:

```elixir
# Normal start - all hardware processes
BB.Supervisor.start_link(MyRobot)

# Simulation mode - actuators replaced with simulators
BB.Supervisor.start_link(MyRobot, simulation: :kinematic)
```

In simulation mode:
- Real actuators are replaced with `BB.Sim.Actuator`
- Controllers can be omitted, mocked, or started normally
- The tree structure remains the same

## Implications for Design

Understanding the supervision architecture helps you design better robots:

### Co-locate Related Processes

Put actuators and sensors for the same joint at the same level:

```elixir
joint :shoulder do
  actuator :servo, {...}          # Same supervisor
  sensor :position, {...}          # Same supervisor
end
```

They restart together if the joint supervisor restarts.

### Separate Independent Subsystems

Put independent subsystems under different parents:

```elixir
sensors do
  sensor :battery, {...}          # Robot-level, independent
end

topology do
  link :base do
    joint :pan do ... end         # Camera pan
    joint :tilt do ... end        # Camera tilt
  end
end
```

Battery monitoring doesn't need to restart when camera joints fail.

### Consider Restart Impact

If a process might crash frequently:
- Put it deep in the tree (affects fewer siblings)
- Give it its own supervisor with appropriate strategy
- Consider whether siblings should restart with it

## Comparison with Alternatives

### Flat Supervision

Some systems use a flat supervisor for all processes:

```
FlatSupervisor
├── Process1
├── Process2
├── Process3
└── ...
```

Problems:
- No fault isolation
- All-or-nothing restart
- Hard to reason about dependencies

### Manual Hierarchies

You could define supervision manually, but:
- Must keep it in sync with physical structure
- Easy to get wrong
- More boilerplate

BB's approach derives the tree automatically from the DSL, ensuring consistency.

## Related Documentation

- [First Robot](../tutorials/01-first-robot.md) - Defining topology
- [Starting and Stopping](../tutorials/02-starting-and-stopping.md) - Working with the supervision tree
- [Understanding Safety](understanding-safety.md) - How safety interacts with supervision
