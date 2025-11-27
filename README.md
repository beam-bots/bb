# Kinetix

Kinetix is a framework for building resilient robotics projects in Elixir.

## Status

No work has started yet.  TODO's include:

- `Kinetix.Message` protocol for serialisation and deserialisation of messages.
- `Kinetix.PubSub` cluster-aware pubsub mechanism for kinetic messages.
- `Kinetix.Topology` extensible DSL for describing robot toplogies.

Additional packages will likely be needed:

- `kinetix_sitl` simulation of Kinetix systems inside Gazebo, etc.
- `kinetix_rc_servo` subscriber which drives PWM-based RC-style servos.
- `kinetix_mavlink` mavlink bridge.
- `kinetix_csrf` crossfire bridge.

## Installation

Kinetix is not yet available in hex, but you can try it by adding it to your project as a Git dependency:

```elixir
def deps do
  [
    {:kinetix, "~> 0.1.0"}
  ]
end
```

