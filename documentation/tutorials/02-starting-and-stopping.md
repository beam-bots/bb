<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Starting and Stopping

In the previous tutorial, we defined a robot using the Beam Bots DSL. Now we'll bring it to life by starting its supervision tree and understanding the process structure.

## Prerequisites

Complete [Your First Robot](01-first-robot.md) first. You should have a `MyRobot` module defined.

## Starting the Robot

Start your robot with `BB.Supervisor.start_link/2`:

```elixir
iex> {:ok, pid} = BB.Supervisor.start_link(MyRobot)
{:ok, #PID<0.234.0>}
```

Your robot is now running. The supervisor has spawned a tree of processes that mirrors your robot's physical structure.

## Understanding the Process Tree

BB creates a supervision tree that reflects your robot's topology:

```
BB.Supervisor (MyRobot)
├── Registry           - Process name registry
├── PubSub Registry    - Message routing
├── Task.Supervisor    - Command execution
├── Runtime            - State machine & robot state
└── LinkSupervisor (:base)
    └── JointSupervisor (:pan_joint)
        └── LinkSupervisor (:pan_link)
            └── JointSupervisor (:tilt_joint)
                └── LinkSupervisor (:camera_link)
```

Each link and joint in your robot definition becomes a supervisor in the process tree.

> **For Roboticists:** A supervisor is like a watchdog process. If a child process crashes, the supervisor can restart it automatically. This is how Erlang/Elixir applications achieve fault tolerance.

> **For Elixirists:** The tree structure mirrors the physical robot. If an actuator on the camera fails, only the camera's subtree is affected - the pan joint and base keep running.

## Fault Isolation

The topology-based supervision gives you fault isolation for free. Consider this scenario:

1. A sensor on `camera_link` crashes due to a hardware glitch
2. Only the `camera_link` supervisor restarts that sensor
3. The rest of the robot continues operating

If the camera link's supervisor itself fails repeatedly:

1. It escalates to its parent (`tilt_joint` supervisor)
2. The tilt joint subtree restarts
3. The pan joint and base continue operating

This mirrors how physical robot failures propagate - a broken wrist doesn't stop the shoulder from working.

## Viewing the Process Tree

You can inspect the running processes:

```elixir
iex> Supervisor.which_children(MyRobot)
[
  {{BB.LinkSupervisor, :base}, #PID<0.236.0>, :supervisor, ...},
  {BB.Robot.Runtime, #PID<0.235.0>, :worker, ...},
  ...
]
```

Or use Observer for a graphical view:

```elixir
iex> :observer.start()
```

Navigate to the Applications tab and find your robot's supervision tree.

## Stopping the Robot

Stop the robot by stopping its supervisor:

```elixir
iex> Supervisor.stop(MyRobot)
:ok
```

This gracefully shuts down all child processes in reverse order.

## Adding to Your Application

In a real application, you'll want to start the robot as part of your application supervision tree.

In your `application.ex`:

```elixir
defmodule MyApp.Application do
  use Application

  @impl true
  def start(_type, _args) do
    children = [
      # Start the robot supervisor
      {BB.Supervisor, MyRobot}
    ]

    opts = [strategy: :one_for_one, name: MyApp.Supervisor]
    Supervisor.start_link(children, opts)
  end
end
```

Now your robot starts automatically with your application.

## Multiple Robots

You can run multiple robots in the same application:

```elixir
children = [
  {BB.Supervisor, LeftArm},
  {BB.Supervisor, RightArm},
  {BB.Supervisor, MobileBase}
]
```

Each robot has its own isolated supervision tree.

## The Robot Runtime

The `Runtime` process manages your robot's operational state. When the robot starts, it's in the `:disarmed` state - a safe mode where actuators won't respond to commands.

```elixir
iex> BB.Robot.Runtime.get_state(MyRobot)
:disarmed
```

We'll cover the state machine and commands in [Commands and State Machine](05-commands.md).

## Process Registration

Every process in the robot tree is registered with a unique name. You can look up any process:

```elixir
iex> BB.Process.whereis(MyRobot, :pan_joint)
#PID<0.238.0>
```

This registry is used internally for routing messages and looking up components.

## Supervision Strategies

By default, BB uses `:one_for_one` supervision - if a child crashes, only that child restarts. This is appropriate for most robotics applications where components are independent.

You can customise the supervisor module in your robot's settings:

```elixir
settings do
  supervisor_module(MySupervisor)
end
```

## What's Next?

The robot is running but not doing much yet. In the next tutorial, we'll:

- Add sensors that publish data
- Subscribe to sensor messages
- Understand the PubSub system

Continue to [Sensors and PubSub](03-sensors-and-pubsub.md).
