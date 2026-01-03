# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.LifecycleTest do
  @moduledoc """
  Integration tests for the complete robot lifecycle: arm → move → disarm.
  """
  use ExUnit.Case, async: true

  alias BB.Error.State.NotAllowed, as: StateError
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  defmodule MoveCommand do
    @moduledoc false
    use BB.Command

    @impl BB.Command
    def handle_command(goal, context, state) do
      positions =
        goal
        |> Enum.into(%{})
        |> Map.take([:shoulder, :elbow])

      :ok = RobotState.set_positions(context.robot_state, positions)

      new_positions = RobotState.get_all_positions(context.robot_state)
      {:stop, :normal, %{state | result: {:ok, new_positions}}}
    end

    @impl BB.Command
    def result(%{result: result}), do: result
  end

  defmodule SimpleArm do
    @moduledoc false
    use BB
    import BB.Unit

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :disarm do
        handler BB.Command.Disarm
        allowed_states [:idle]
      end

      command :move do
        handler BB.Robot.LifecycleTest.MoveCommand
        allowed_states [:idle]
      end
    end

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          origin do
            z(~u(0.1 meter))
          end

          axis do
          end

          limit do
            lower(~u(-90 degree))
            upper(~u(90 degree))
            effort(~u(50 newton_meter))
            velocity(~u(2 radian_per_second))
          end

          link :upper_arm do
            joint :elbow do
              type :revolute

              origin do
                z(~u(0.5 meter))
              end

              axis do
              end

              limit do
                lower(~u(0 degree))
                upper(~u(135 degree))
                effort(~u(30 newton_meter))
                velocity(~u(3 radian_per_second))
              end

              link :forearm do
              end
            end
          end
        end
      end
    end
  end

  describe "robot lifecycle" do
    test "robot starts in disarmed state" do
      start_supervised!(SimpleArm)

      assert Runtime.state(SimpleArm) == :disarmed
    end

    test "arm command transitions from disarmed to idle" do
      start_supervised!(SimpleArm)

      assert Runtime.state(SimpleArm) == :disarmed

      {:ok, cmd} = SimpleArm.arm()
      assert {:ok, :armed, _opts} = BB.Command.await(cmd)

      assert Runtime.state(SimpleArm) == :idle
    end

    test "disarm command transitions from idle to disarmed" do
      start_supervised!(SimpleArm)

      {:ok, cmd} = SimpleArm.arm()
      BB.Command.await(cmd)
      assert Runtime.state(SimpleArm) == :idle

      {:ok, cmd} = SimpleArm.disarm()
      assert {:ok, :disarmed, _opts} = BB.Command.await(cmd)

      assert Runtime.state(SimpleArm) == :disarmed
    end

    test "move command updates joint positions" do
      start_supervised!(SimpleArm)

      {:ok, cmd} = SimpleArm.arm()
      BB.Command.await(cmd)

      {:ok, cmd} = SimpleArm.move(shoulder: 0.5, elbow: 1.0)
      {:ok, positions} = BB.Command.await(cmd)

      assert_in_delta positions.shoulder, 0.5, 0.001
      assert_in_delta positions.elbow, 1.0, 0.001

      assert Runtime.state(SimpleArm) == :idle
    end

    test "move command rejected when disarmed" do
      start_supervised!(SimpleArm)

      assert Runtime.state(SimpleArm) == :disarmed

      # Commands that can't start return error directly
      assert {:error, %StateError{current_state: :disarmed}} = SimpleArm.move(shoulder: 0.5)
    end

    test "arm command rejected when already armed" do
      start_supervised!(SimpleArm)

      {:ok, cmd} = SimpleArm.arm()
      BB.Command.await(cmd)

      # Commands that can't start return error directly
      assert {:error, %StateError{current_state: :idle}} = SimpleArm.arm()
    end

    test "disarm command rejected when disarmed" do
      start_supervised!(SimpleArm)

      # Commands that can't start return error directly
      assert {:error, %StateError{current_state: :disarmed}} = SimpleArm.disarm()
    end

    test "full lifecycle: arm → move → move → disarm" do
      start_supervised!(SimpleArm)

      assert Runtime.state(SimpleArm) == :disarmed

      {:ok, cmd} = SimpleArm.arm()
      assert {:ok, :armed, _opts} = BB.Command.await(cmd)
      assert Runtime.state(SimpleArm) == :idle

      {:ok, cmd} = SimpleArm.move(shoulder: 0.3)
      {:ok, positions} = BB.Command.await(cmd)
      assert_in_delta positions.shoulder, 0.3, 0.001
      assert Runtime.state(SimpleArm) == :idle

      {:ok, cmd} = SimpleArm.move(shoulder: 0.6, elbow: 0.8)
      {:ok, positions} = BB.Command.await(cmd)
      assert_in_delta positions.shoulder, 0.6, 0.001
      assert_in_delta positions.elbow, 0.8, 0.001
      assert Runtime.state(SimpleArm) == :idle

      {:ok, cmd} = SimpleArm.disarm()
      assert {:ok, :disarmed, _opts} = BB.Command.await(cmd)
      assert Runtime.state(SimpleArm) == :disarmed
    end
  end

  describe "pubsub integration" do
    alias BB.StateMachine.Transition

    test "broadcasts state transitions during lifecycle" do
      start_supervised!(SimpleArm)

      BB.PubSub.subscribe(SimpleArm, [:state_machine])

      {:ok, cmd} = SimpleArm.arm()
      BB.Command.await(cmd)

      # Safety.Controller publishes arm transition
      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :disarmed, to: :armed}}}
    end

    test "disarm broadcasts transition to disarmed" do
      start_supervised!(SimpleArm)

      {:ok, cmd} = SimpleArm.arm()
      BB.Command.await(cmd)

      BB.PubSub.subscribe(SimpleArm, [:state_machine])

      {:ok, cmd} = SimpleArm.disarm()
      BB.Command.await(cmd)

      # Runtime publishes command start transition
      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :idle, to: :executing}}}

      # Safety.Controller publishes disarm transitions (during command execution)
      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :armed, to: :disarming}}}

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :disarming, to: :disarmed}}}

      # Runtime publishes command completion (to :disarmed due to next_state option)
      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :executing, to: :disarmed}}}
    end
  end
end
