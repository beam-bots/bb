# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.AddressingTest do
  use ExUnit.Case

  alias BB.Message
  alias BB.Message.Actuator.Command

  defmodule Robot do
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

          link :arm do
            joint :elbow do
              type :revolute

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(180 degree_per_second))
              end

              actuator :forearm_motor, BB.Test.RecordingActuator

              link :hand
            end
          end
        end
      end
    end
  end

  setup do
    :persistent_term.put({BB.Test.RecordingActuator, Robot}, self())
    start_supervised!(Robot)
    :ok = BB.Safety.arm(Robot)
    on_exit(fn -> :persistent_term.erase({BB.Test.RecordingActuator, Robot}) end)
    :ok
  end

  describe "BB.Robot.actuator_path/2" do
    test "resolves an actuator directly under the root link's joint" do
      assert BB.Robot.actuator_path(Robot.robot(), :motor) == {:ok, [:base, :shoulder, :motor]}
    end

    test "resolves an actuator nested deeper in the topology" do
      assert BB.Robot.actuator_path(Robot.robot(), :forearm_motor) ==
               {:ok, [:base, :shoulder, :arm, :elbow, :forearm_motor]}
    end

    test "reports an unknown actuator as an actuator, not a link" do
      assert {:error, %BB.Error.Kinematics.UnknownActuator{actuator: :nonexistent}} =
               BB.Robot.actuator_path(Robot.robot(), :nonexistent)
    end

    test "agrees with the path the framework injects into the driver" do
      # The resolved path must equal the actuator's `bb.path`, since that is what
      # its server derives its command topic from. If these ever diverge, pubsub
      # commanding silently stops working — which is the bug this all came from.
      :ok = BB.publish(Robot, [:actuator | [:base, :shoulder, :motor]], position(0.1))
      assert_receive {:received, :command, %Message{payload: %Command.Position{}}}, 500
    end
  end

  describe "addressing by name" do
    test "set_position/4 accepts a bare actuator name" do
      :ok = BB.Actuator.set_position(Robot, :motor, 0.5)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end

    test "a name resolves to the same actuator as its full path" do
      :ok = BB.Actuator.set_position(Robot, :forearm_motor, 0.25)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.25}}},
                     500
    end

    test "the full path still works" do
      :ok = BB.Actuator.set_position(Robot, [:base, :shoulder, :motor], 0.75)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.75}}},
                     500
    end

    test "stop/3 and hold/3 accept a name too" do
      :ok = BB.Actuator.stop(Robot, :motor)
      assert_receive {:received, :command, %Message{payload: %Command.Stop{}}}, 500

      :ok = BB.Actuator.hold(Robot, :motor)
      assert_receive {:received, :command, %Message{payload: %Command.Hold{}}}, 500
    end
  end

  describe "addressing by path on the direct transports" do
    test "set_position!/4 accepts a full path" do
      :ok = BB.Actuator.set_position!(Robot, [:base, :shoulder, :motor], 0.5)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end

    test "set_position_sync/5 accepts a full path" do
      assert {:ok, :accepted} =
               BB.Actuator.set_position_sync(Robot, [:base, :shoulder, :motor], 0.5, [], 500)

      assert_receive {:received, :command, %Message{payload: %Command.Position{position: 0.5}}},
                     500
    end
  end

  describe "unknown actuators" do
    test "raise rather than publishing to a topic nothing listens on" do
      assert_raise ArgumentError, ~r/has no actuator named :nonexistent/, fn ->
        BB.Actuator.set_position(Robot, :nonexistent, 0.5)
      end
    end

    test "the error names the actuators that do exist" do
      error =
        assert_raise ArgumentError, fn ->
          BB.Actuator.set_position(Robot, :nonexistent, 0.5)
        end

      assert Exception.message(error) =~ ":motor"
      assert Exception.message(error) =~ ":forearm_motor"
    end
  end

  defp position(value), do: Message.new!(Command.Position, :motor, position: value)
end
