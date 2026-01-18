<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Deploy to Nerves

Deploy your Beam Bots robot to embedded hardware using Nerves.

## Prerequisites

- Nerves tooling installed (see [Nerves Installation](https://hexdocs.pm/nerves/installation.html))
- Supported hardware (Raspberry Pi, BeagleBone, etc.)

## Step 1: Create a Nerves Project

Generate a new Nerves project:

```bash
mix nerves.new my_robot_firmware
cd my_robot_firmware
```

## Step 2: Install Beam Bots with Igniter

Use the Igniter installer to add Beam Bots:

```bash
mix igniter.install bb
```

This will:
- Add `bb` to your dependencies
- Create a `MyRobotFirmware.Robot` module with arm/disarm commands and a base link
- Add the robot to your application supervision tree
- Configure the formatter for the BB DSL

## Step 3: Add Hardware Drivers

Add servo driver dependencies for your hardware. For example, with a PCA9685 PWM controller:

```bash
mix igniter.install bb_servo_pca9685
```

Or manually add to `mix.exs`:

```elixir
defp deps do
  [
    # ... existing deps ...
    {:bb_servo_pca9685, "~> 0.1"}
  ]
end
```

Then run `mix deps.get`.

## Step 4: Configure Your Robot

Edit the generated robot module to add your hardware configuration:

```elixir
# lib/my_robot_firmware/robot.ex
defmodule MyRobotFirmware.Robot do
  use BB

  # Add controller for your servo driver
  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      joint :pan, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        # Add actuator and sensor for each joint
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :position, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
      end

      joint :tilt, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}
        sensor :position, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
      end
    end
  end
end
```

## Step 5: Configure Hardware

Set up hardware-specific configuration in your Nerves config:

```elixir
# config/target.exs
config :nerves, :firmware, rootfs_overlay: "rootfs_overlay"
```

If using GPIO (e.g., for `bb_servo_pigpio`):

```elixir
config :pigpiox,
  gpio_port: 8888
```

## Step 6: Build and Deploy

Build the firmware:

```bash
export MIX_TARGET=rpi4  # or your target
mix deps.get
mix firmware
```

Deploy to device:

```bash
# First time (burn to SD card)
mix burn

# Updates over network
mix upload my_robot.local
```

## Step 7: Connect and Control

SSH into the device:

```bash
ssh my_robot.local
```

In IEx:

```elixir
alias MyRobotFirmware.Robot

# Arm the robot
{:ok, cmd} = Robot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Move joints
BB.Actuator.set_position!(Robot, :servo, 0.5)
```

## Network Control

### With bb_liveview

Add the LiveView dashboard for web-based control:

```bash
mix igniter.install bb_liveview
```

Then configure your router:

```elixir
# In router
import BB.LiveView.Router
scope "/" do
  pipe_through :browser
  bb_dashboard "/", MyRobotFirmware.Robot
end
```

Access at `http://my_robot.local/`.

### With Custom GenServer

Create a TCP/UDP server for remote control:

```elixir
defmodule MyRobotFirmware.RemoteControl do
  use GenServer

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  def init(_opts) do
    {:ok, socket} = :gen_tcp.listen(4000, [:binary, active: true, reuseaddr: true])
    {:ok, %{socket: socket}}
  end

  def handle_info({:tcp, _socket, data}, state) do
    case Jason.decode(data) do
      {:ok, %{"command" => "move", "joint" => joint, "position" => pos}} ->
        BB.Actuator.set_position!(MyRobotFirmware.Robot, String.to_atom(joint), pos)

      _ ->
        :ok
    end

    {:noreply, state}
  end
end
```

## Hardware Watchdog

For safety-critical applications, add a hardware watchdog:

```elixir
defmodule MyRobotFirmware.Watchdog do
  use GenServer

  @heartbeat_pin 18
  @interval_ms 100

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  def init(_opts) do
    {:ok, gpio} = Circuits.GPIO.open(@heartbeat_pin, :output)
    schedule_heartbeat()
    {:ok, %{gpio: gpio}}
  end

  def handle_info(:heartbeat, state) do
    Circuits.GPIO.write(state.gpio, 1)
    Process.sleep(1)
    Circuits.GPIO.write(state.gpio, 0)
    schedule_heartbeat()
    {:noreply, state}
  end

  defp schedule_heartbeat do
    Process.send_after(self(), :heartbeat, @interval_ms)
  end
end
```

## Common Issues

### I2C Device Not Found

Check the bus name matches your hardware:
- Raspberry Pi: Usually `"i2c-1"`
- BeagleBone: May be `"i2c-2"`

Verify with:

```elixir
Circuits.I2C.detect_devices("i2c-1")
```

### GPIO Permissions

Ensure GPIO is accessible. On some targets, add udev rules or use the `nerves_runtime` GPIO interface.

### Network Not Available on Boot

The robot may start before network is ready. If using network control, handle connection failures gracefully:

```elixir
def init(_opts) do
  # Wait for network
  VintageNet.subscribe(["interface", "eth0", "connection"])
  {:ok, :waiting_for_network}
end

def handle_info({VintageNet, ["interface", "eth0", "connection"], _, :internet, _}, state) do
  # Network ready, start accepting connections
  {:noreply, start_server(state)}
end
```

## Testing Locally

Test your robot code on your development machine before deploying:

```elixir
# Start in simulation mode
{:ok, _} = BB.Supervisor.start_link(MyRobotFirmware.Robot, simulation: :kinematic)
```

## Adding More Robots

To add additional robots to the same firmware:

```bash
mix bb.add_robot --robot MyRobotFirmware.Robots.SecondRobot
```

## Next Steps

- Add [bb_liveview](https://hexdocs.pm/bb_liveview) for web-based control
- Implement [hardware safety](../topics/understanding-safety.md) with watchdog
- Consider OTA updates with NervesHub
