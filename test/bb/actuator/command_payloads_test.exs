# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.CommandPayloadsTest do
  use ExUnit.Case

  alias BB.Error.State.UnsupportedCommand
  alias BB.Message
  alias BB.Message.Actuator.Command

  defmodule Bespoke do
    @moduledoc false
    defstruct [:pattern]

    use BB.Message,
      schema: [
        pattern: [type: :atom, required: true, doc: "Which canned pattern to run"]
      ]
  end

  # A driver whose hardware speaks a command BB doesn't model. Without
  # `command_payloads/1` it would have to subscribe to its own command topic,
  # and the message would arrive having skipped the arm check.
  defmodule WidenedActuator do
    @moduledoc false
    use BB.Actuator, options_schema: []

    @impl BB.Actuator
    def command_payloads(_opts),
      do: [BB.Actuator.CommandPayloadsTest.Bespoke | BB.Actuator.default_command_payloads()]

    @impl BB.Actuator
    def init(opts) do
      bb = Keyword.fetch!(opts, :bb)
      {:ok, %{recipient: :persistent_term.get({__MODULE__, bb.robot}, nil)}}
    end

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    @impl BB.Actuator
    def handle_command(%Message{} = message, state) do
      if state.recipient, do: send(state.recipient, {:commanded, message.payload})
      {:noreply, state}
    end
  end

  # A port that only speaks effort. Everything else is refused by the framework
  # rather than handed to the driver to misinterpret.
  defmodule NarrowedActuator do
    @moduledoc false
    use BB.Actuator, options_schema: []

    @impl BB.Actuator
    def command_payloads(_opts), do: [Command.Effort]

    @impl BB.Actuator
    def init(opts) do
      bb = Keyword.fetch!(opts, :bb)
      {:ok, %{recipient: :persistent_term.get({__MODULE__, bb.robot}, nil)}}
    end

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    @impl BB.Actuator
    def handle_command(%Message{} = message, state) do
      if state.recipient, do: send(state.recipient, {:commanded, message.payload})
      {:noreply, state}
    end
  end

  defmodule Robot do
    @moduledoc false
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit effort: ~u(10 newton_meter), velocity: ~u(180 degree_per_second)

          actuator :wide, BB.Actuator.CommandPayloadsTest.WidenedActuator
          actuator :narrow, BB.Actuator.CommandPayloadsTest.NarrowedActuator

          link :arm do
          end
        end
      end
    end
  end

  setup do
    :persistent_term.put({WidenedActuator, Robot}, self())
    :persistent_term.put({NarrowedActuator, Robot}, self())
    start_supervised!(Robot)
    :ok = BB.Safety.arm(Robot)

    on_exit(fn ->
      :persistent_term.erase({WidenedActuator, Robot})
      :persistent_term.erase({NarrowedActuator, Robot})
    end)

    :ok
  end

  describe "the default" do
    test "is the six built-in command payloads" do
      assert BB.Actuator.default_command_payloads() == [
               Command.Effort,
               Command.Hold,
               Command.Position,
               Command.Stop,
               Command.Trajectory,
               Command.Velocity
             ]
    end
  end

  describe "widening" do
    test "a bespoke payload reaches handle_command/2 through the pipeline" do
      message = Message.new!(Bespoke, :wide, pattern: :wave)
      :ok = BB.publish(Robot, [:actuator, :base, :shoulder, :wide], message)

      assert_receive {:commanded, %Bespoke{pattern: :wave}}, 500
    end

    test "and is refused while disarmed, like any other command" do
      :ok = BB.Safety.disarm(Robot)

      message = Message.new!(Bespoke, :wide, pattern: :wave)
      :ok = BB.publish(Robot, [:actuator, :base, :shoulder, :wide], message)

      refute_receive {:commanded, _}, 200
    end

    test "the built-ins still work alongside it" do
      assert :ok = BB.Actuator.set_position(Robot, :wide, 0.5)
      assert_receive {:commanded, %Command.Position{position: 0.5}}, 500
    end
  end

  describe "narrowing" do
    test "an accepted payload gets through" do
      :ok = BB.Actuator.set_effort(Robot, :narrow, 1.5)
      assert_receive {:commanded, %Command.Effort{}}, 500
    end

    test "a payload outside the list never arrives, and the caller is told so" do
      assert {:error, %UnsupportedCommand{actuator: :narrow, command: Command.Position}} =
               BB.Actuator.set_position(Robot, :narrow, 0.5, timeout: 500)

      refute_receive {:commanded, _}, 200
    end

    test "narrowing holds on the direct transport, not just the published one" do
      :ok = BB.Actuator.set_position(Robot, :narrow, 0.5, delivery: :direct)
      refute_receive {:commanded, _}, 200
    end

    test "the error names what the actuator does accept" do
      {:error, error} = BB.Actuator.set_position(Robot, :narrow, 0.5, timeout: 500)
      assert Exception.message(error) =~ "Command.Effort"
    end

    test "Stop is refused too — nothing is exempt from the declared list" do
      # Stop is a motion command, not a safety mechanism, so it has no special
      # standing here. A driver that never declared it can't be handed it.
      :ok = BB.Actuator.stop(Robot, :narrow)
      refute_receive {:commanded, _}, 200
    end

    test "and is refused on the direct transport with a structured error" do
      assert {:error, %UnsupportedCommand{command: Command.Stop}} =
               BB.Actuator.stop_sync(Robot, :narrow, [], 500)
    end

    test "refusal is observable" do
      handler = {__MODULE__, :unsupported, self()}

      :telemetry.attach(
        handler,
        [:bb, :actuator, :rejected],
        fn _event, _measurements, metadata, pid -> send(pid, {:telemetry, metadata}) end,
        self()
      )

      on_exit(fn -> :telemetry.detach(handler) end)

      :ok = BB.Actuator.set_position(Robot, :narrow, 0.5, delivery: :direct)

      assert_receive {:telemetry, %{reason: :unsupported_command, actuator: :narrow}}, 500
    end
  end
end
