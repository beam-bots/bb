<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Exporting to URDF

In this tutorial, you'll learn how to export your Kinetix robot definition to URDF format for use with external tools.

## Prerequisites

Complete [Your First Robot](01-first-robot.md). You should have a robot module defined.

## What is URDF?

> **For Elixirists:** URDF (Unified Robot Description Format) is an XML format for describing robots. It's the standard in the ROS ecosystem and supported by visualisation tools like RViz and simulators like Gazebo.

URDF describes:
- Links (rigid bodies) with visual and collision geometry
- Joints connecting links with motion constraints
- Physical properties (mass, inertia)
- Materials and colours

Exporting to URDF lets you visualise your Kinetix robots in established tools.

## Using the Mix Task

Export your robot with the `kinetix.to_urdf` mix task:

```bash
# Print URDF to stdout
mix kinetix.to_urdf MyRobot

# Write to a file
mix kinetix.to_urdf MyRobot --output robot.urdf

# Short form
mix kinetix.to_urdf MyRobot -o robot.urdf
```

## Example Output

For a simple two-joint robot, the output looks like:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot name="MyRobot">
  <link name="base">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.05"/>
      </geometry>
      <material name="base_material">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <link name="pan_link">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
    </visual>
  </link>

  <joint name="pan_joint" type="revolute">
    <parent link="base"/>
    <child link="pan_link"/>
    <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    <axis xyz="0.0 0.0 1.0"/>
    <limit lower="-1.5708" upper="1.5708" effort="5.0" velocity="1.0472"/>
  </joint>

  <!-- ... more links and joints ... -->
</robot>
```

## Programmatic Export

You can also export from Elixir code:

```elixir
alias Kinetix.Urdf.Exporter

# From a module
{:ok, xml} = Exporter.export(MyRobot)

# From a robot struct
robot = MyRobot.robot()
{:ok, xml} = Exporter.export_robot(robot)

# Write to file
File.write!("robot.urdf", xml)
```

## Viewing in RViz

If you have ROS installed, view your robot:

```bash
# Export the robot
mix kinetix.to_urdf MyRobot -o robot.urdf

# View in RViz (requires ROS)
roslaunch urdf_tutorial display.launch model:=robot.urdf
```

## Loading in Gazebo

For simulation in Gazebo:

```bash
# Export
mix kinetix.to_urdf MyRobot -o robot.urdf

# Launch Gazebo with the model
gazebo --verbose robot.urdf
```

Note: Gazebo may require additional tags for physics simulation (like `<inertial>` on all links).

## What Gets Exported

The exporter converts these Kinetix elements to URDF:

| Kinetix | URDF |
|---------|------|
| `link` | `<link>` |
| `joint` (revolute, prismatic, etc.) | `<joint>` |
| `visual` with geometry | `<visual>` |
| `collision` | `<collision>` |
| `inertial` (mass, inertia) | `<inertial>` |
| `material` and `color` | `<material>` |
| `origin` | `<origin>` |
| `axis` | `<axis>` |
| `limit` | `<limit>` |
| `dynamics` | `<dynamics>` |

## Working with Meshes

If your robot uses mesh geometry:

```elixir
visual do
  mesh do
    filename "meshes/arm_link.stl"
    scale 0.001  # Convert mm to metres
  end
end
```

The URDF will reference the same path:

```xml
<geometry>
  <mesh filename="meshes/arm_link.stl" scale="0.001"/>
</geometry>
```

Make sure the mesh files are available relative to where you'll use the URDF.

## Limitations

Some Kinetix features don't map directly to URDF:

| Feature | Status |
|---------|--------|
| Sensors | Not exported (URDF extension) |
| Actuators | Not exported (URDF extension) |
| Commands | Not exported (Kinetix-specific) |
| Controllers | Not exported (Kinetix-specific) |
| `floating` joints | Exported but limited support |
| `planar` joints | Exported but limited support |

URDF is primarily a static description format. Dynamic elements like sensors and controllers are typically added through separate configuration in tools like ROS.

## Unit Conversion

Kinetix automatically converts units to URDF conventions:

| Quantity | URDF Unit |
|----------|-----------|
| Position | metres |
| Angle | radians |
| Mass | kilograms |
| Force | newtons |
| Torque | newton-metres |

Your `~u()` values are converted automatically:

```elixir
# In Kinetix
limit do
  lower(~u(-90 degree))
  upper(~u(90 degree))
end

# In URDF
<limit lower="-1.5708" upper="1.5708" .../>
```

## Validation Tips

After exporting, validate your URDF:

```bash
# Using ROS tools
check_urdf robot.urdf

# Or view the structure
urdf_to_graphiz robot.urdf
```

Common issues:
- Missing `<inertial>` on links (required for simulation)
- Mesh file paths not found
- Joint limits in wrong order (lower > upper)

## Round-Trip Workflow

A typical development workflow:

1. Define robot in Kinetix (Elixir DSL)
2. Export to URDF for visualisation
3. Test kinematics in RViz
4. Run runtime in Kinetix (supervision, sensors, commands)
5. Re-export after changes

The URDF serves as a visualisation and validation tool, while Kinetix handles the runtime.

## Summary

You've completed the Kinetix tutorials! You now know how to:

1. Define robots using the DSL
2. Start and stop supervision trees
3. Add sensors and subscribe to messages
4. Compute forward kinematics
5. Implement and execute commands
6. Export to URDF for external tools

## Next Steps

- Explore the [DSL Reference](../dsls/DSL-Kinetix.md) for all available options
- Check the module documentation for API details
- Look at the example robots in `test/support/example_robots.ex`
