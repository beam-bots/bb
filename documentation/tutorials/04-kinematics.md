# Forward Kinematics

In this tutorial, you'll learn how to compute link positions from joint angles using Kinetix's forward kinematics system.

## Prerequisites

Complete [Your First Robot](01-first-robot.md). You should have a `MyRobot` module with at least two joints.

## What is Forward Kinematics?

> **For Elixirists:** Forward kinematics answers the question "if my joints are at these angles, where is my end effector?" It's the mathematical relationship between joint angles and Cartesian positions.

Forward kinematics computes the position and orientation of any link given the current joint positions. For a robot arm:

- **Input:** Joint angles (e.g., shoulder at 45°, elbow at 30°)
- **Output:** Position of the hand in 3D space (x, y, z)

Kinetix uses 4x4 homogeneous transformation matrices internally, leveraging Nx tensors for efficient computation.

## Computing Link Position

Pass a map of joint positions to compute where a link is:

```elixir
iex> robot = MyRobot.robot()
iex> alias Kinetix.Robot.Kinematics

iex> positions = %{
...>   pan_joint: :math.pi() / 4,
...>   tilt_joint: :math.pi() / 6
...> }

iex> {x, y, z} = Kinematics.link_position(robot, positions, :camera_link)
{0.021213203435596423, 0.021213203435596423, 0.10598076211353316}
```

The result is in metres, relative to the base link's origin.

Positions are in **radians** for revolute joints and **metres** for prismatic joints.

> **Tip:** Use `:math.pi()` for readable angle values. π/4 = 45°, π/2 = 90°, etc.

## Querying a Running Robot

For a running robot, query the Runtime for current joint positions:

```elixir
iex> {:ok, _} = Kinetix.Supervisor.start_link(MyRobot)
iex> positions = Kinetix.Robot.Runtime.positions(MyRobot)
%{pan_joint: 0.0, tilt_joint: 0.0}

iex> robot = MyRobot.robot()
iex> {x, y, z} = Kinematics.link_position(robot, positions, :camera_link)
```

The Runtime maintains joint positions based on sensor feedback. See [Commands and State Machine](05-commands.md) for how sensors update positions.

## Getting the Full Transform

For orientation as well as position, get the full 4x4 transform:

```elixir
iex> positions = %{pan_joint: :math.pi() / 4, tilt_joint: 0.0}
iex> transform = Kinematics.forward_kinematics(robot, positions, :camera_link)
#Nx.Tensor<
  f64[4][4]
  ...
>
```

Extract components:

```elixir
iex> alias Kinetix.Robot.Transform

# Get position
iex> {x, y, z} = Transform.get_translation(transform)

# Get rotation matrix (3x3)
iex> rotation = Transform.get_rotation(transform)
```

## All Link Transforms

Compute transforms for every link at once:

```elixir
iex> positions = %{pan_joint: :math.pi() / 4, tilt_joint: :math.pi() / 6}
iex> transforms = Kinematics.all_link_transforms(robot, positions)
%{
  base: #Nx.Tensor<...>,
  pan_link: #Nx.Tensor<...>,
  camera_link: #Nx.Tensor<...>
}
```

This is more efficient than calling `forward_kinematics/3` multiple times.

## Working with Transforms

Transforms are 4x4 homogeneous matrices. Here are common operations:

```elixir
alias Kinetix.Robot.Transform

# Identity transform (no translation, no rotation)
identity = Transform.identity()

# Create from position
t1 = Transform.from_translation(0.1, 0.0, 0.5)

# Create rotation around Z axis
t2 = Transform.from_rotation_z(:math.pi() / 4)

# Compose transforms (apply t2 then t1)
combined = Transform.compose(t1, t2)

# Invert a transform
inverse = Transform.inverse(transform)
```

> **For Roboticists:** These are standard SE(3) operations. The transforms follow the DH convention internally but are exposed through a cleaner API.

## Practical Example: Sweeping Joint Angles

Here's a complete example that tracks the camera position as we sweep through joint angles:

```elixir
defmodule KinematicsDemo do
  alias Kinetix.Robot.Kinematics

  def sweep_pan(robot) do
    # Sweep pan joint from -90° to +90°
    for angle <- -90..90//10 do
      radians = angle * :math.pi() / 180
      positions = %{pan_joint: radians, tilt_joint: 0.0}

      {x, y, z} = Kinematics.link_position(robot, positions, :camera_link)
      IO.puts("Pan #{angle}°: x=#{Float.round(x, 3)}, y=#{Float.round(y, 3)}, z=#{Float.round(z, 3)}")
    end
  end
end

# Run it
robot = MyRobot.robot()
KinematicsDemo.sweep_pan(robot)
```

Output:

```
Pan -90°: x=-0.03, y=0.0, z=0.08
Pan -80°: x=-0.03, y=0.005, z=0.08
...
Pan 0°: x=0.0, y=0.03, z=0.08
...
Pan 90°: x=0.03, y=0.0, z=0.08
```

## Unit Conventions

Kinetix uses SI units throughout:

| Quantity | Unit |
|----------|------|
| Position | metres |
| Angle | radians |
| Velocity | m/s or rad/s |
| Force | newtons |
| Torque | newton-metres |

This differs from the DSL where you can use degrees and other units. The `~u()` sigil handles conversion automatically.

## Coordinate Frames

Each link has its own coordinate frame. The transform returned by `forward_kinematics/3` describes how to get from the base frame to the link's frame.

For a pan-tilt camera:
- **base** frame: fixed to the world
- **pan_link** frame: rotates with the pan joint
- **camera_link** frame: rotates with both pan and tilt

When the camera looks at a point, you need to transform that point from world coordinates into the camera's frame.

## What's Next?

Forward kinematics tells you where links are given joint angles. But how do you control the robot? In the next tutorial, we'll:

- Understand the robot state machine (disarmed → idle → executing)
- Use built-in arm/disarm commands
- Implement custom commands

Continue to [Commands and State Machine](05-commands.md).
