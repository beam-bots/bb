# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.ServerCommandPipelineTest do
  use ExUnit.Case

  import ExUnit.CaptureLog

  alias BB.Error.State.NotArmed
  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command

  defmodule ArmedRobot do
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          actuator :motor, BB.Test.RecordingActuator

          link :arm
        end
      end
    end
  end

  defmodule DisarmedRobot do
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          actuator :motor, BB.Test.RecordingActuator

          link :arm
        end
      end
    end
  end

  defmodule SlowRobot do
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          actuator :motor, {BB.Test.RecordingActuator, delay_ms: 300}

          link :arm
        end
      end
    end
  end

  defmodule SubscribingRobot do
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          actuator :motor, {BB.Test.RecordingActuator, subscribe_to: [:sensor, :probe]} do
            transmission do
              reduction 50.0
            end
          end

          link :arm
        end
      end
    end
  end

  @actuator_topic [:actuator, :base, :shoulder, :motor]

  defp start_robot(robot_module) do
    :persistent_term.put({BB.Test.RecordingActuator, robot_module}, self())
    start_supervised!(robot_module)
    on_exit(fn -> :persistent_term.erase({BB.Test.RecordingActuator, robot_module}) end)
  end

  defp position(value), do: Message.new!(Command.Position, :motor, position: value)

  describe "transports" do
    setup do
      start_robot(ArmedRobot)
      :ok = BB.Safety.arm(ArmedRobot)
      :ok
    end

    test "a published command reaches the driver, which never subscribed itself" do
      :ok = BB.publish(ArmedRobot, @actuator_topic, position(0.5))

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end

    test "set_position/4 reaches the driver via the actuator's full path" do
      :ok = BB.Actuator.set_position(ArmedRobot, [:base, :shoulder, :motor], 0.25)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.25}}},
                     500
    end

    test "set_position/4 still publishes the command for observers" do
      BB.subscribe(ArmedRobot, @actuator_topic)

      :ok = BB.Actuator.set_position(ArmedRobot, [:base, :shoulder, :motor], 0.25)

      assert_receive {:bb, @actuator_topic, %Message{payload: %Command.Position{position: 0.25}}},
                     500
    end

    test "set_position/4 drives the actuator once, not once per transport" do
      :ok = BB.Actuator.set_position(ArmedRobot, [:base, :shoulder, :motor], 0.25)

      assert_receive {:received, :command, %Message{payload: %Command.Position{}}}, 500
      refute_receive {:received, :command, %Message{payload: %Command.Position{}}}, 200
    end

    test "delivery: :direct reaches the driver without waiting" do
      :ok = BB.Actuator.set_position(ArmedRobot, :motor, 0.25, delivery: :direct)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.25}}},
                     500
    end

    test "a cast command reaches the driver" do
      :ok = BB.cast(ArmedRobot, :motor, {:command, position(0.5)})

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end

    test "a call command reaches the driver and is acknowledged" do
      assert {:ok, :accepted} =
               BB.call(ArmedRobot, :motor, {:command, position(0.5)}, 500)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end

    test "the actuator does not receive its own outbound BeginMotion" do
      begin_motion =
        Message.new!(BeginMotion, :shoulder,
          initial_position: 0.0,
          target_position: 1.0,
          expected_arrival: System.monotonic_time(:millisecond) + 100
        )

      :ok = BB.publish(ArmedRobot, @actuator_topic, begin_motion)

      refute_receive {:received, _kind, %Message{payload: %BeginMotion{}}}, 200
    end
  end

  describe "waiting" do
    setup do
      start_robot(SlowRobot)
      :ok = BB.Safety.arm(SlowRobot)
      :ok
    end

    test ":timeout bounds how long the caller waits for a slow driver" do
      assert {:timeout, _} =
               catch_exit(BB.Actuator.set_position(SlowRobot, :motor, 0.5, timeout: 50))
    end

    test "and the default is long enough for one that merely takes its time" do
      assert :ok = BB.Actuator.set_position(SlowRobot, :motor, 0.5)
    end
  end

  describe "arm gating" do
    setup do
      start_robot(DisarmedRobot)
      :ok
    end

    test "a published command is dropped while disarmed" do
      :ok = BB.publish(DisarmedRobot, @actuator_topic, position(0.5))

      refute_receive {:received, :command, _message}, 200
    end

    test "set_position/4 reports the refusal rather than leaving the caller guessing" do
      assert {:error, %NotArmed{actuator: :motor, command: Command.Position}} =
               BB.Actuator.set_position(DisarmedRobot, :motor, 0.5, timeout: 500)

      refute_receive {:received, :command, _message}, 200
    end

    test "a refusal is logged, whichever transport delivered the command" do
      log =
        capture_log(fn ->
          :ok = BB.Actuator.set_position(DisarmedRobot, :motor, 0.5, delivery: :direct)
          refute_receive {:received, :command, _message}, 200
        end)

      assert log =~ "refused"
      assert log =~ "robot is not armed"
      assert log =~ ":motor"
    end

    test "a cast command is dropped while disarmed" do
      :ok = BB.cast(DisarmedRobot, :motor, {:command, position(0.5)})

      refute_receive {:received, :command, _message}, 200
    end

    test "a call command is refused with a structured error while disarmed" do
      assert {:error, %NotArmed{actuator: :motor, command: Command.Position}} =
               BB.call(DisarmedRobot, :motor, {:command, position(0.5)}, 500)

      refute_receive {:received, :command, _message}, 200
    end

    test "Stop is refused while disarmed, like every other command" do
      # Stop means "cease travelling and go passive", not "make the hardware
      # safe" — that's `disarm`. A disarmed actuator isn't being driven, so
      # there's nothing for it to do, and no reason to exempt it from the gate.
      :ok = BB.cast(DisarmedRobot, :motor, {:command, Message.new!(Command.Stop, :motor, [])})

      refute_receive {:received, :command, _message}, 200
    end

    test "Hold is refused while disarmed too" do
      :ok = BB.cast(DisarmedRobot, :motor, {:command, Message.new!(Command.Hold, :motor, [])})

      refute_receive {:received, :command, _message}, 200
    end

    test "commands flow once the robot is armed" do
      :ok = BB.Safety.arm(DisarmedRobot)
      :ok = BB.publish(DisarmedRobot, @actuator_topic, position(0.5))

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end

    test "a rejected command emits telemetry" do
      handler = {__MODULE__, :rejected, self()}

      :telemetry.attach(
        handler,
        [:bb, :actuator, :rejected],
        fn event, measurements, metadata, pid ->
          send(pid, {:telemetry, event, measurements, metadata})
        end,
        self()
      )

      on_exit(fn -> :telemetry.detach(handler) end)

      :ok = BB.cast(DisarmedRobot, :motor, {:command, position(0.5)})

      assert_receive {:telemetry, [:bb, :actuator, :rejected], %{count: 1}, metadata}, 500
      assert metadata.actuator == :motor
      assert metadata.transport == :cast
      assert metadata.reason == :disarmed
      assert metadata.payload_module == Command.Position
    end
  end

  describe "the driver's own subscriptions" do
    setup do
      start_robot(SubscribingRobot)
      :ok = BB.Safety.arm(SubscribingRobot)
      :ok
    end

    test "arrive in handle_info/2 without the transmission applied" do
      :ok = BB.publish(SubscribingRobot, [:sensor, :probe], position(0.5))

      assert_receive {:received, :info, %Message{payload: %Command.Position{position: 0.5}}}, 500
      refute_receive {:received, :command, _message}, 200
    end

    test "while commands on its own topic still get the transmission" do
      :ok = BB.publish(SubscribingRobot, @actuator_topic, position(0.5))

      assert_receive {:received, :command, %Message{payload: %Command.Position{} = cmd}}, 500
      assert_in_delta cmd.position, 25.0, 1.0e-9
    end
  end
end
