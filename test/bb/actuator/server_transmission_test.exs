# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.ServerTransmissionTest do
  use ExUnit.Case

  alias BB.Message
  alias BB.Message.Actuator.Command

  defmodule WithTransmission do
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          transmission do
            reduction 50.0
            offset(~u(45 degree))
            reversed? true
          end

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

  defmodule WithoutTransmission do
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

  @transmission %{reduction: 50.0, offset: :math.pi() / 4, reversed?: true}

  defp start_robot(robot_module) do
    :persistent_term.put({BB.Test.RecordingActuator, robot_module}, self())
    start_supervised!(robot_module)
    on_exit(fn -> :persistent_term.erase({BB.Test.RecordingActuator, robot_module}) end)
  end

  describe "with a transmission" do
    setup do
      start_robot(WithTransmission)
      :ok
    end

    test "transforms a Position command from joint-space to motor-space (pubsub)" do
      :ok =
        BB.publish(
          WithTransmission,
          [:actuator, :base, :shoulder, :motor],
          Message.new!(Command.Position, :motor, position: :math.pi() / 4 + 0.01)
        )

      assert_receive {:received, :info, %Message{payload: %Command.Position{} = cmd}}, 500

      expected = BB.Transmission.apply_position(:math.pi() / 4 + 0.01, @transmission)
      assert_in_delta cmd.position, expected, 1.0e-9
    end

    test "transforms a Position command via cast" do
      message = Message.new!(Command.Position, :motor, position: :math.pi() / 4 + 0.01)
      :ok = BB.cast(WithTransmission, :motor, {:command, message})

      assert_receive {:received, :cast, %Message{payload: %Command.Position{} = cmd}}, 500
      expected = BB.Transmission.apply_position(:math.pi() / 4 + 0.01, @transmission)
      assert_in_delta cmd.position, expected, 1.0e-9
    end

    test "transforms a Position command via call" do
      message = Message.new!(Command.Position, :motor, position: :math.pi() / 4 + 0.01)
      {:ok, :accepted} = BB.call(WithTransmission, :motor, {:command, message}, 500)

      assert_receive {:received, :call, %Message{payload: %Command.Position{} = cmd}}, 500
      expected = BB.Transmission.apply_position(:math.pi() / 4 + 0.01, @transmission)
      assert_in_delta cmd.position, expected, 1.0e-9
    end

    test "transforms the velocity hint on a Position command" do
      message =
        Message.new!(Command.Position, :motor, position: :math.pi() / 4, velocity: 0.1)

      :ok = BB.cast(WithTransmission, :motor, {:command, message})

      assert_receive {:received, :cast, %Message{payload: %Command.Position{} = cmd}}, 500
      assert_in_delta cmd.velocity, -50.0 * 0.1, 1.0e-9
    end

    test "passes Hold commands through unchanged" do
      hold = Message.new!(Command.Hold, :motor, [])
      :ok = BB.cast(WithTransmission, :motor, {:command, hold})

      assert_receive {:received, :cast, %Message{payload: %Command.Hold{}}}, 500
    end
  end

  describe "without a transmission" do
    setup do
      start_robot(WithoutTransmission)
      :ok
    end

    test "position commands flow through unchanged" do
      message = Message.new!(Command.Position, :motor, position: 1.23)
      :ok = BB.cast(WithoutTransmission, :motor, {:command, message})

      assert_receive {:received, :cast, %Message{payload: %Command.Position{position: p}}}, 500
      assert p == 1.23
    end
  end
end
