<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Use URDF with ROS Tools

Export your BB robot to URDF format for use with ROS visualisation and simulation tools.

## Prerequisites

- A BB robot module (see [First Robot](../tutorials/01-first-robot.md))
- ROS 2 installed (for RViz, Gazebo)
- Understanding of the [URDF Export Tutorial](../tutorials/06-urdf-export.md)

## Step 1: Export to URDF

Use the mix task to export:

```bash
# Print to stdout
mix bb.to_urdf MyRobot

# Write to file
mix bb.to_urdf MyRobot -o robot.urdf
```

Or programmatically:

```elixir
urdf = BB.URDF.Exporter.export(MyRobot)
File.write!("robot.urdf", urdf)
```

## Step 2: Validate the URDF

Use `check_urdf` from ROS:

```bash
check_urdf robot.urdf
```

Expected output:

```
robot name is: MyRobot
---------- Successfully Parsed XML ---------------
root Link: base has 1 child(ren)
    child(1):  link_1
        child(1):  link_2
```

## Step 3: View in RViz

### Create a Launch File

```python
# robot_display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'robot.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config/robot.rviz']
        )
    ])
```

### Run RViz

```bash
ros2 launch my_robot_pkg robot_display.launch.py
```

In RViz:
1. Add a RobotModel display
2. Set the Description Topic to `/robot_description`
3. Set Fixed Frame to your base link

## Step 4: Publish Joint States from BB

Bridge BB joint states to ROS:

```elixir
defmodule MyRobot.ROSBridge do
  use GenServer

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  def init(opts) do
    robot_module = opts[:robot]
    BB.subscribe(robot_module, [:sensor])

    # Connect to ROS (using your preferred ROS Elixir bridge)
    {:ok, ros} = ROS.connect()

    {:ok, %{robot: robot_module, ros: ros}}
  end

  def handle_info({:bb, [:sensor, _joint], %{payload: joint_state}}, state) do
    # Convert to ROS JointState message
    ros_msg = %{
      header: %{stamp: ROS.now(), frame_id: ""},
      name: Enum.map(joint_state.names, &Atom.to_string/1),
      position: joint_state.positions,
      velocity: joint_state.velocities,
      effort: joint_state.efforts
    }

    ROS.publish(state.ros, "/joint_states", ros_msg)
    {:noreply, state}
  end
end
```

## Step 5: Use with Gazebo

### Add Gazebo Plugins

BB's URDF export creates basic structure. For Gazebo simulation, add plugins:

```xml
<!-- Add to robot.urdf -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>

<gazebo reference="shoulder">
  <material>Gazebo/Orange</material>
</gazebo>
```

### Launch in Gazebo

```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -file robot.urdf -entity my_robot
```

## Step 6: Bidirectional Control

### BB → ROS (Publish States)

```elixir
# In your ROS bridge
def handle_info({:bb, [:sensor, _], %{payload: js}}, state) do
  ROS.publish(state.ros, "/joint_states", to_ros_joint_state(js))
  {:noreply, state}
end
```

### ROS → BB (Receive Commands)

```elixir
def init(opts) do
  # ...
  ROS.subscribe(state.ros, "/joint_commands", &handle_ros_command/1)
  {:ok, state}
end

defp handle_ros_command(msg) do
  for {name, position} <- Enum.zip(msg.name, msg.position) do
    joint = String.to_atom(name)
    BB.Actuator.set_position!(MyRobot, joint, position)
  end
end
```

## URDF Limitations

BB's URDF export has limitations compared to hand-written URDF:

| Feature | BB Support | Notes |
|---------|------------|-------|
| Links | ✓ | Position from transforms |
| Joints | ✓ | Type, limits, axis |
| Visual geometry | ✓ | Basic shapes only |
| Collision geometry | ✓ | Same as visual |
| Inertial | ✓ | Mass and inertia tensor |
| Transmissions | ✗ | Add manually for ros_control |
| Gazebo plugins | ✗ | Add manually |
| Materials | ✗ | Gazebo materials need manual addition |

## Adding Missing Elements

For features BB doesn't export, post-process the URDF:

```elixir
defmodule URDFEnhancer do
  def add_gazebo_materials(urdf) do
    urdf
    |> String.replace(
      "</robot>",
      """
      <gazebo reference="base">
        <material>Gazebo/Grey</material>
      </gazebo>
      </robot>
      """
    )
  end

  def add_transmission(urdf, joint_name) do
    transmission = """
    <transmission name="#{joint_name}_transmission">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="#{joint_name}">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      </joint>
      <actuator name="#{joint_name}_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
    """

    String.replace(urdf, "</robot>", transmission <> "</robot>")
  end
end
```

## MoveIt Integration

For motion planning with MoveIt:

1. Export URDF from BB
2. Create MoveIt config package:
   ```bash
   ros2 run moveit_setup_assistant moveit_setup_assistant
   ```
3. Load your URDF in the assistant
4. Configure:
   - Planning groups
   - End effectors
   - Self-collision matrix
5. Generate the config package

## Common Issues

### Joint Names Don't Match

BB uses atoms for joint names. Ensure ROS messages use the string version:

```elixir
Atom.to_string(:shoulder)  #=> "shoulder"
```

### Frame ID Mismatch

BB uses frame_id in messages. Map to ROS TF frame names:

```elixir
def to_ros_frame(bb_frame_id) do
  "#{@robot_name}/#{bb_frame_id}"
end
```

### Units

BB uses SI units (radians, metres). ROS also uses SI, so no conversion needed.

## Next Steps

- Add visual meshes for realistic rendering
- Configure ros_control for hardware interface
- Set up MoveIt for motion planning
