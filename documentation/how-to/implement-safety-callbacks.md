<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# How to Implement Safety Callbacks

Implement `disarm/1` callbacks for actuators and controllers that control physical hardware.

## Prerequisites

- Understanding of the BB safety system (see [Understanding Safety](../topics/understanding-safety.md))
- An actuator or controller that controls physical hardware

## The Core Requirement

The `disarm/1` callback must make hardware safe **without access to GenServer state**. This is critical because the callback may be invoked after your process has crashed.

## Step 1: Register with the Safety Controller

In your `init/1`, register with the safety controller and provide all options needed for stateless disarm:

```elixir
defmodule MyActuator do
  use GenServer
  use BB.Actuator

  @impl GenServer
  def init({bb, opts}) do
    # Register with safety controller
    BB.Safety.register(__MODULE__,
      robot: bb.robot_module,
      path: bb.path,
      opts: [
        # Include everything disarm/1 needs
        pin: opts[:pin],
        bus: opts[:bus],
        address: opts[:address]
      ]
    )

    {:ok, %{bb: bb, opts: opts}}
  end
end
```

## Step 2: Implement the disarm/1 Callback

The callback receives only the options you provided at registration:

```elixir
@impl BB.Actuator
def disarm(opts) do
  # opts contains: :robot, :path, and your custom :opts
  pin = opts[:opts][:pin]
  bus = opts[:opts][:bus]

  # Make hardware safe - this must work even if the actuator process is dead
  case connect_and_disable(bus, pin) do
    :ok -> :ok
    {:error, reason} -> {:error, reason}
  end
end

defp connect_and_disable(bus, pin) do
  # Open a fresh connection - don't rely on cached references
  {:ok, device} = SomeHardware.open(bus)
  SomeHardware.set_output(device, pin, 0)
  SomeHardware.close(device)
  :ok
end
```

## Common Patterns

### GPIO-based servos (e.g., pigpio)

```elixir
@impl BB.Actuator
def disarm(opts) do
  pin = opts[:opts][:pin]

  # Open fresh GPIO connection
  {:ok, gpio} = Pigpio.connect()
  Pigpio.set_servo_pulsewidth(gpio, pin, 0)
  :ok
rescue
  _ -> {:error, :gpio_connection_failed}
end
```

### I2C-based controllers (e.g., PCA9685)

```elixir
@impl BB.Actuator
def disarm(opts) do
  channel = opts[:opts][:channel]
  controller_name = opts[:opts][:controller]

  # Get controller process (it might still be alive)
  case BB.Process.whereis(opts[:robot], controller_name) do
    {:ok, pid} ->
      # Use controller to disable channel
      GenServer.call(pid, {:disable_channel, channel})

    {:error, _} ->
      # Controller is dead - connect directly to hardware
      bus = opts[:opts][:bus]
      address = opts[:opts][:address]
      direct_disable(bus, address, channel)
  end
end

defp direct_disable(bus, address, channel) do
  {:ok, ref} = Circuits.I2C.open(bus)
  # Write directly to PCA9685 registers to disable channel
  Circuits.I2C.write(ref, address, <<0x06 + channel * 4, 0, 0, 0, 0>>)
  Circuits.I2C.close(ref)
  :ok
end
```

### Serial-based servos (e.g., Dynamixel)

```elixir
@impl BB.Actuator
def disarm(opts) do
  servo_id = opts[:opts][:servo_id]
  port = opts[:opts][:port]
  baud = opts[:opts][:baud]

  # Open fresh serial connection
  {:ok, uart} = Circuits.UART.start_link()
  :ok = Circuits.UART.open(uart, port, speed: baud)

  # Send torque disable command
  packet = Robotis.Protocol.V2.write_packet(servo_id, 64, <<0>>)
  Circuits.UART.write(uart, packet)

  Circuits.UART.close(uart)
  GenServer.stop(uart)
  :ok
end
```

## Step 3: Handle Registration Options

Pass all hardware-specific options needed for disarm:

```elixir
# For an I2C servo
BB.Safety.register(__MODULE__,
  robot: bb.robot_module,
  path: bb.path,
  opts: [
    channel: opts[:channel],
    controller: opts[:controller],
    # Fallback for direct hardware access
    bus: opts[:bus] || "i2c-1",
    address: opts[:address] || 0x40
  ]
)

# For a GPIO servo
BB.Safety.register(__MODULE__,
  robot: bb.robot_module,
  path: bb.path,
  opts: [
    pin: opts[:pin],
    gpio_host: opts[:gpio_host] || "localhost"
  ]
)
```

## Testing Safety Callbacks

Test that disarm works after process crash:

```elixir
defmodule MyActuator.SafetyTest do
  use ExUnit.Case
  use Mimic

  setup :verify_on_exit!

  test "disarm works after actuator crash" do
    # Start robot
    {:ok, sup} = BB.Supervisor.start_link(MyRobot)
    {:ok, cmd} = MyRobot.arm()
    BB.Command.await(cmd)

    # Get actuator pid
    {:ok, actuator_pid} = BB.Process.whereis(MyRobot, [:joint, :servo])

    # Expect disarm to be called
    expect(SomeHardware, :set_output, fn _device, _pin, 0 -> :ok end)

    # Kill the actuator
    Process.exit(actuator_pid, :kill)

    # Disarm should still work
    assert :ok = BB.Safety.disarm(MyRobot)
  end

  test "disarm callback can access hardware directly" do
    # Test the callback in isolation
    opts = %{
      robot: MyRobot,
      path: [:joint, :servo],
      opts: [pin: 18, bus: "i2c-1"]
    }

    expect(SomeHardware, :open, fn "i2c-1" -> {:ok, :mock_device} end)
    expect(SomeHardware, :set_output, fn :mock_device, 18, 0 -> :ok end)
    expect(SomeHardware, :close, fn :mock_device -> :ok end)

    assert :ok = MyActuator.disarm(opts)
  end
end
```

## Error Handling

If disarm fails, the robot enters `:error` state:

```elixir
@impl BB.Actuator
def disarm(opts) do
  case attempt_disarm(opts) do
    :ok ->
      :ok

    {:error, reason} ->
      # Log the failure - operator will need to manually intervene
      Logger.error("Failed to disarm #{inspect(opts[:path])}: #{inspect(reason)}")
      {:error, reason}
  end
end
```

Recovery from `:error` state requires manual intervention:

```elixir
# After fixing the hardware issue
BB.Safety.force_disarm(MyRobot)
```

## Common Mistakes

### Relying on process state

```elixir
# BAD - state is not available in disarm/1
def disarm(_opts) do
  Pigpio.set_servo_pulsewidth(@gpio_ref, @pin, 0)  # Module attributes won't help
end

# GOOD - use only opts
def disarm(opts) do
  {:ok, gpio} = Pigpio.connect()
  Pigpio.set_servo_pulsewidth(gpio, opts[:opts][:pin], 0)
end
```

### Caching hardware references

```elixir
# BAD - cached reference may be stale
def init(opts) do
  {:ok, gpio} = Pigpio.connect()
  BB.Safety.register(__MODULE__, ..., opts: [gpio: gpio])  # Reference won't survive crash
end

# GOOD - open fresh connection in disarm
def disarm(opts) do
  {:ok, gpio} = Pigpio.connect(opts[:opts][:host])
  # ...
end
```

### Not handling connection failures

```elixir
# BAD - crash on connection failure
def disarm(opts) do
  {:ok, device} = SomeHardware.open(opts[:opts][:bus])
  # ...
end

# GOOD - handle failures gracefully
def disarm(opts) do
  case SomeHardware.open(opts[:opts][:bus]) do
    {:ok, device} ->
      SomeHardware.disable(device)
      :ok

    {:error, reason} ->
      {:error, {:connection_failed, reason}}
  end
end
```

## Verification Checklist

Before deploying your actuator:

- [ ] `disarm/1` works without GenServer state
- [ ] `disarm/1` opens fresh hardware connections
- [ ] `disarm/1` handles connection failures gracefully
- [ ] Test passes after killing actuator process
- [ ] All hardware-specific options are passed at registration
- [ ] Timeout is considered (5 second limit per callback)

## Next Steps

- Understand the full safety system in [Understanding Safety](../topics/understanding-safety.md)
- Learn about [Hardware Error Reporting](../topics/understanding-safety.md#hardware-error-reporting)
