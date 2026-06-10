<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Starting and Stopping

In the previous tutorial, we defined a robot using the Beam Bots DSL. Now we'll bring it to life and understand the supervision tree that runs it.

## Prerequisites

Complete [Your First Robot](01-first-robot.md) first. You should have a `MyRobot.Robot` module defined.

## Your Robot Starts With Your Application

When you installed Beam Bots, the installer added your robot to your application's supervision tree. You can see it in `lib/my_robot/application.ex`:

```elixir
defmodule MyRobot.Application do
  use Application

  @impl true
  def start(_type, _args) do
    children = [
      {MyRobot.Robot, robot_opts()}
    ]

    opts = [strategy: :one_for_one, name: MyRobot.Supervisor]
    Supervisor.start_link(children, opts)
  end

  defp robot_opts do
    if System.get_env("SIMULATE") do
      [simulation: :kinematic]
    else
      Application.get_env(:my_robot, MyRobot.Robot, [])
    end
  end
end
```

So your robot starts automatically whenever your application boots. The generated `robot_opts/0` decides how: set the `SIMULATE` environment variable to boot in kinematic simulation (see [Simulation Mode](10-simulation.md)), otherwise it reads any startup options you've placed in config (see [Parameters](07-parameters.md)). Launch an IEx session with your project loaded:

```sh
iex -S mix
```

Your robot is already running. The supervisor has spawned a tree of processes that mirrors your robot's physical structure. Confirm it's alive by asking for its state:

```elixir
iex> BB.Robot.Runtime.state(MyRobot.Robot)
:disarmed
```

> **Note:** Because the robot is already supervised, calling `MyRobot.Robot.start_link/1` yourself returns `{:error, {:already_started, pid}}` rather than starting a second copy. That's expected — there is one robot, and it is already running. See [Starting a Robot Manually](#starting-a-robot-manually) if you want to control startup yourself.

## Understanding the Process Tree

BB creates a supervision tree that reflects your robot's topology:

```
BB.Supervisor (MyRobot.Robot)
├── Registry              - Process name registry
├── PubSub Registry       - Message routing
├── Task.Supervisor       - Command execution
├── Runtime               - State machine & robot state
├── BridgeSupervisor      - External communication (not hardware)
└── TopologySupervisor    - Hardware-facing subtree (its own restart budget)
    ├── SensorSupervisor      - Robot-level sensors
    ├── ControllerSupervisor  - Robot-level controllers
    └── LinkSupervisor (:base_link)
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

If failures cascade all the way up and exhaust the topology supervisor's own restart budget (configurable via `topology_max_restarts` and `topology_max_seconds`), the safety controller force-disarms the robot and transitions it to `:error`. Infrastructure (Runtime, PubSub, bridges) keeps running so external systems can observe the failure and acknowledge with `BB.Safety.force_disarm/1`.

## Viewing the Process Tree

You can inspect the running processes:

```elixir
iex> Supervisor.which_children(MyRobot.Robot)
[
  {{BB.LinkSupervisor, :base_link}, #PID<0.236.0>, :supervisor, ...},
  {BB.Robot.Runtime, #PID<0.235.0>, :worker, ...},
  ...
]
```

Or use Observer for a graphical view:

```elixir
iex> :observer.start()
```

Navigate to the Applications tab and find your robot's supervision tree.

## Stopping and Restarting

Your robot is a permanent child of your application's supervisor, so stopping it directly just triggers an immediate restart:

```elixir
iex> Supervisor.stop(MyRobot.Robot)
:ok
iex> BB.Robot.Runtime.state(MyRobot.Robot)
:disarmed  # MyRobot.Supervisor has already restarted it
```

To stop it and keep it stopped — while experimenting, say — terminate it through its parent supervisor:

```elixir
iex> Supervisor.terminate_child(MyRobot.Supervisor, MyRobot.Robot)
:ok
```

Start it again with:

```elixir
iex> Supervisor.restart_child(MyRobot.Supervisor, MyRobot.Robot)
{:ok, #PID<0.260.0>}
```

Stopping a supervisor gracefully shuts down all of its child processes in reverse order.

## Starting a Robot Manually

If a robot is *not* part of a supervision tree — in a script, a test, or a fresh `iex` session started without your application — start it yourself:

```elixir
iex> {:ok, pid} = MyRobot.Robot.start_link()
{:ok, #PID<0.234.0>}
```

`MyRobot.Robot.start_link/1` accepts the same options as its child spec, such as `params:` and `simulation:`:

```elixir
iex> {:ok, pid} = MyRobot.Robot.start_link(simulation: :kinematic)
```

## Multiple Robots

You can run multiple robots in the same application:

```elixir
children = [
  LeftArm,
  RightArm,
  MobileBase
]
```

Each robot has its own isolated supervision tree.

## The Robot Runtime

The `Runtime` process manages your robot's operational state. When the robot starts, it's in the `:disarmed` state - a safe mode where actuators won't respond to commands.

```elixir
iex> BB.Robot.Runtime.state(MyRobot.Robot)
:disarmed
```

We'll cover the state machine and commands in [Commands and State Machine](05-commands.md).

## Process Registration

Every process in the robot tree is registered with a unique name. You can look up any process:

```elixir
iex> BB.Process.whereis(MyRobot.Robot, :pan_joint)
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
