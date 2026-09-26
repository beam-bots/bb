# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.DeliveryTest do
  use ExUnit.Case

  alias BB.Error.State.NotArmed
  alias BB.Error.State.StaleEpoch
  alias BB.Error.State.UnsupportedCommand
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Test.Commands
  alias Spark.Options.ValidationError

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

          link :arm
        end
      end
    end
  end

  # A port that only speaks effort, so every other payload is refused before
  # the driver sees it — one of the three refusals `authorise/2` can produce.
  defmodule EffortOnlyActuator do
    @moduledoc false
    use BB.Actuator, options_schema: []

    @impl BB.Actuator
    def command_payloads(_opts), do: [Command.Effort]

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    @impl BB.Actuator
    def capabilities(_opts), do: [:position_feedback]

    @impl BB.Actuator
    def init(_opts), do: {:ok, %{}}

    @impl BB.Actuator
    def handle_command(_message, state), do: {:noreply, state}
  end

  defmodule NarrowRobot do
    use BB

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          actuator :motor, BB.Actuator.DeliveryTest.EffortOnlyActuator

          link :arm
        end
      end
    end
  end

  @actuator_topic [:actuator, :base, :shoulder, :motor]

  @waypoints [[position: 0.1, time_from_start: 100], [position: 0.2, time_from_start: 200]]

  # Every command type: the `BB.Actuator` function that sends it, the arguments
  # it takes between the target and the options, and the payload the actuator
  # should end up seeing.
  @commands [
    {"position", Command.Position, :set_position, [0.5]},
    {"velocity", Command.Velocity, :set_velocity, [0.25]},
    {"effort", Command.Effort, :set_effort, [1.5]},
    {"trajectory", Command.Trajectory, :follow_trajectory, [@waypoints]},
    {"stop", Command.Stop, :stop, []},
    {"hold", Command.Hold, :hold, []}
  ]

  defp send_command(robot, fun, args, opts),
    do: apply(BB.Actuator, fun, [robot, :motor] ++ args ++ [opts])

  defp start_robot(robot_module) do
    :persistent_term.put({BB.Test.RecordingActuator, robot_module}, self())
    start_supervised!(robot_module)
    on_exit(fn -> :persistent_term.erase({BB.Test.RecordingActuator, robot_module}) end)
  end

  describe "every command type accepts every delivery" do
    setup do
      start_robot(Robot)
      :ok = BB.Safety.arm(Robot)
      :ok
    end

    for {name, payload, fun, args} <- @commands do
      test "#{name} reaches the driver under :pubsub" do
        assert :ok = send_command(Robot, unquote(fun), unquote(args), delivery: :pubsub)
        assert_receive {:received, :command, %Message{payload: %unquote(payload){}}}, 500
      end

      test "#{name} reaches the driver under :broadcast" do
        assert :ok = send_command(Robot, unquote(fun), unquote(args), delivery: :broadcast)
        assert_receive {:received, :command, %Message{payload: %unquote(payload){}}}, 500
      end

      test "#{name} reaches the driver under :direct" do
        assert :ok = send_command(Robot, unquote(fun), unquote(args), delivery: :direct)
        assert_receive {:received, :command, %Message{payload: %unquote(payload){}}}, 500
      end

      test "#{name} publishes under :pubsub, for observers other than the actuator" do
        BB.subscribe(Robot, @actuator_topic)

        assert :ok = send_command(Robot, unquote(fun), unquote(args), delivery: :pubsub)
        assert_receive {:bb, @actuator_topic, %Message{payload: %unquote(payload){}}}, 500
      end

      test "#{name} publishes under :broadcast" do
        BB.subscribe(Robot, @actuator_topic)

        assert :ok = send_command(Robot, unquote(fun), unquote(args), delivery: :broadcast)
        assert_receive {:bb, @actuator_topic, %Message{payload: %unquote(payload){}}}, 500
      end

      test "#{name} publishes nothing under :direct" do
        BB.subscribe(Robot, @actuator_topic)

        assert :ok = send_command(Robot, unquote(fun), unquote(args), delivery: :direct)
        refute_receive {:bb, @actuator_topic, _}, 200
      end
    end
  end

  describe "defaults are preserved per command type" do
    setup do
      start_robot(Robot)
      :ok = BB.Safety.arm(Robot)
      BB.subscribe(Robot, @actuator_topic)
      :ok
    end

    # `set_position/4` is the one command that waited for its actuator before
    # the delivery options were unified, and it still does.
    test "set_position defaults to :pubsub, so it waits and can report a refusal" do
      assert :ok = BB.Actuator.set_position(Robot, :motor, 0.5)
      assert_receive {:bb, @actuator_topic, %Message{payload: %Command.Position{}}}, 500

      :ok = BB.Safety.disarm(Robot)
      assert {:error, %NotArmed{}} = BB.Actuator.set_position(Robot, :motor, 0.5)
    end

    for {name, payload, fun, args} <- @commands, name != "position" do
      test "#{name} defaults to :broadcast, publishing without waiting" do
        assert :ok = send_command(Robot, unquote(fun), unquote(args), [])
        assert_receive {:bb, @actuator_topic, %Message{payload: %unquote(payload){}}}, 500
      end

      test "#{name} returns :ok under its default even when the robot is disarmed" do
        :ok = BB.Safety.disarm(Robot)
        assert :ok = send_command(Robot, unquote(fun), unquote(args), [])
      end
    end
  end

  describe "reply_on_reject? rejects the deliveries it cannot serve" do
    setup do
      start_robot(Robot)
      :ok = BB.Safety.arm(Robot)
      :ok
    end

    for {name, _payload, fun, args} <- @commands do
      test "#{name} raises under :pubsub" do
        assert_raise ValidationError, ~r/reply_on_reject\?/, fn ->
          send_command(Robot, unquote(fun), unquote(args),
            delivery: :pubsub,
            reply_on_reject?: true
          )
        end
      end

      test "#{name} raises under :broadcast" do
        assert_raise ValidationError, ~r/reply_on_reject\?/, fn ->
          send_command(Robot, unquote(fun), unquote(args),
            delivery: :broadcast,
            reply_on_reject?: true
          )
        end
      end

      test "#{name} accepts it under :direct" do
        assert :ok =
                 send_command(Robot, unquote(fun), unquote(args),
                   delivery: :direct,
                   reply_on_reject?: true
                 )
      end
    end

    test "the :pubsub message says the refusal is already returned" do
      error =
        assert_raise ValidationError, fn ->
          BB.Actuator.set_position(Robot, :motor, 0.5,
            delivery: :pubsub,
            reply_on_reject?: true
          )
        end

      assert Exception.message(error) =~ "delivery: :pubsub"
      assert Exception.message(error) =~ "delivery: :direct"
    end

    test "the :broadcast message says there is no one caller to answer" do
      error =
        assert_raise ValidationError, fn ->
          BB.Actuator.set_position(Robot, :motor, 0.5,
            delivery: :broadcast,
            reply_on_reject?: true
          )
        end

      assert Exception.message(error) =~ "delivery: :broadcast"
      assert Exception.message(error) =~ "every subscriber"
    end

    test "explicitly asking for the default is not a combination error" do
      assert :ok =
               BB.Actuator.set_position(Robot, :motor, 0.5,
                 delivery: :pubsub,
                 reply_on_reject?: false
               )
    end

    test "a non-boolean is refused by the schema" do
      assert_raise ValidationError, ~r/reply_on_reject\?/, fn ->
        BB.Actuator.set_position(Robot, :motor, 0.5,
          delivery: :direct,
          reply_on_reject?: :yes
        )
      end
    end
  end

  describe "reply_on_reject? reports every refusal the pipeline can produce" do
    setup do
      start_robot(Robot)
      :ok
    end

    test "a disarmed robot" do
      command_id = make_ref()

      assert :ok =
               BB.Actuator.set_position(Robot, :motor, 0.5,
                 delivery: :direct,
                 command_id: command_id,
                 reply_on_reject?: true
               )

      assert_receive {:bb, :command_rejected, :motor, ^command_id, %NotArmed{}}, 500
    end

    test "a stale arm epoch" do
      :ok = BB.Safety.arm(Robot)
      :ok = BB.Safety.disarm(Robot)
      :ok = BB.Safety.arm(Robot)

      {:ok, current} = BB.Safety.epoch(Robot)
      command_id = make_ref()

      stale = %{
        Message.new!(Command.Position, :motor, position: 0.5, command_id: command_id)
        | arm_epoch: current - 1
      }

      :ok = Commands.cast(Robot, :motor, stale, self())

      assert_receive {:bb, :command_rejected, :motor, ^command_id, %StaleEpoch{}}, 500
    end

    test "a payload the driver never declared" do
      start_supervised!(NarrowRobot)
      :ok = BB.Safety.arm(NarrowRobot)
      command_id = make_ref()

      assert :ok =
               BB.Actuator.set_position(NarrowRobot, :motor, 0.5,
                 delivery: :direct,
                 command_id: command_id,
                 reply_on_reject?: true
               )

      assert_receive {:bb, :command_rejected, :motor, ^command_id, %UnsupportedCommand{}}, 500
    end

    test "the reply carries nil when the caller set no command_id" do
      assert :ok =
               BB.Actuator.set_position(Robot, :motor, 0.5,
                 delivery: :direct,
                 reply_on_reject?: true
               )

      assert_receive {:bb, :command_rejected, :motor, nil, %NotArmed{}}, 500
    end

    test "an accepted command is answered with silence" do
      :ok = BB.Safety.arm(Robot)

      assert :ok =
               BB.Actuator.set_position(Robot, :motor, 0.5,
                 delivery: :direct,
                 command_id: make_ref(),
                 reply_on_reject?: true
               )

      assert_receive {:received, :command, %Message{}}, 500
      refute_receive {:bb, :command_rejected, _, _, _}, 200
    end

    test "nothing is sent to a caller that didn't ask" do
      assert :ok = BB.Actuator.set_position(Robot, :motor, 0.5, delivery: :direct)
      refute_receive {:bb, :command_rejected, _, _, _}, 200
    end

    for {name, _payload, fun, args} <- @commands, name != "position" do
      test "#{name} reports a refusal too" do
        assert :ok =
                 send_command(Robot, unquote(fun), unquote(args),
                   delivery: :direct,
                   reply_on_reject?: true
                 )

        assert_receive {:bb, :command_rejected, :motor, nil, %NotArmed{}}, 500
      end
    end
  end

  describe "option validation" do
    setup do
      start_robot(Robot)
      :ok = BB.Safety.arm(Robot)
      :ok
    end

    test "an unknown delivery is refused rather than silently dropping the command" do
      assert_raise ValidationError, ~r/delivery/, fn ->
        BB.Actuator.set_position(Robot, :motor, 0.5, delivery: :carrier_pigeon)
      end
    end

    test "a misspelled option is refused rather than ignored" do
      assert_raise ValidationError, ~r/velocty/, fn ->
        BB.Actuator.set_position(Robot, :motor, 0.5, velocty: 0.5)
      end
    end

    test "an option belonging to another command type is refused" do
      assert_raise ValidationError, ~r/velocity/, fn ->
        BB.Actuator.hold(Robot, :motor, velocity: 0.5)
      end
    end

    test "command_id reaches the driver" do
      command_id = make_ref()
      :ok = BB.Actuator.set_position(Robot, :motor, 0.5, command_id: command_id)

      assert_receive {:received, :command,
                      %Message{payload: %Command.Position{command_id: ^command_id}}},
                     500
    end
  end
end
