# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.LifecycleTest do
  @moduledoc """
  Integration tests for the complete robot lifecycle: arm → move → disarm.
  """
  use ExUnit.Case, async: true

  alias Kinetix.Robot.Runtime
  alias Kinetix.Robot.State, as: RobotState

  defmodule MoveCommand do
    @moduledoc false
    @behaviour Kinetix.Command

    @impl true
    def handle_command(goal, context) do
      positions =
        goal
        |> Enum.into(%{})
        |> Map.take([:shoulder, :elbow])

      :ok = RobotState.set_positions(context.robot_state, positions)

      new_positions = RobotState.get_all_positions(context.robot_state)
      {:ok, new_positions}
    end
  end

  defmodule SimpleArm do
    @moduledoc false
    use Kinetix
    import Kinetix.Unit

    commands do
      command :arm do
        handler Kinetix.Command.Arm
        allowed_states [:disarmed]
      end

      command :disarm do
        handler Kinetix.Command.Disarm
        allowed_states [:idle]
      end

      command :move do
        handler Kinetix.Robot.LifecycleTest.MoveCommand
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
            z(~u(1 meter))
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
                z(~u(1 meter))
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

      {:ok, task} = SimpleArm.arm()
      assert {:ok, :armed} = Task.await(task)

      assert Runtime.state(SimpleArm) == :idle
    end

    test "disarm command transitions from idle to disarmed" do
      start_supervised!(SimpleArm)

      {:ok, task} = SimpleArm.arm()
      Task.await(task)
      assert Runtime.state(SimpleArm) == :idle

      {:ok, task} = SimpleArm.disarm()
      assert {:ok, :disarmed} = Task.await(task)

      assert Runtime.state(SimpleArm) == :disarmed
    end

    test "move command updates joint positions" do
      start_supervised!(SimpleArm)

      {:ok, task} = SimpleArm.arm()
      Task.await(task)

      {:ok, task} = SimpleArm.move(shoulder: 0.5, elbow: 1.0)
      {:ok, positions} = Task.await(task)

      assert_in_delta positions.shoulder, 0.5, 0.001
      assert_in_delta positions.elbow, 1.0, 0.001

      assert Runtime.state(SimpleArm) == :idle
    end

    test "move command rejected when disarmed" do
      start_supervised!(SimpleArm)

      assert Runtime.state(SimpleArm) == :disarmed

      {:ok, task} = SimpleArm.move(shoulder: 0.5)
      assert {:error, %Runtime.StateError{current_state: :disarmed}} = Task.await(task)
    end

    test "arm command rejected when already armed" do
      start_supervised!(SimpleArm)

      {:ok, task} = SimpleArm.arm()
      Task.await(task)

      {:ok, task} = SimpleArm.arm()
      assert {:error, %Runtime.StateError{current_state: :idle}} = Task.await(task)
    end

    test "disarm command rejected when disarmed" do
      start_supervised!(SimpleArm)

      {:ok, task} = SimpleArm.disarm()
      assert {:error, %Runtime.StateError{current_state: :disarmed}} = Task.await(task)
    end

    test "full lifecycle: arm → move → move → disarm" do
      start_supervised!(SimpleArm)

      assert Runtime.state(SimpleArm) == :disarmed

      {:ok, task} = SimpleArm.arm()
      assert {:ok, :armed} = Task.await(task)
      assert Runtime.state(SimpleArm) == :idle

      {:ok, task} = SimpleArm.move(shoulder: 0.3)
      {:ok, positions} = Task.await(task)
      assert_in_delta positions.shoulder, 0.3, 0.001
      assert Runtime.state(SimpleArm) == :idle

      {:ok, task} = SimpleArm.move(shoulder: 0.6, elbow: 0.8)
      {:ok, positions} = Task.await(task)
      assert_in_delta positions.shoulder, 0.6, 0.001
      assert_in_delta positions.elbow, 0.8, 0.001
      assert Runtime.state(SimpleArm) == :idle

      {:ok, task} = SimpleArm.disarm()
      assert {:ok, :disarmed} = Task.await(task)
      assert Runtime.state(SimpleArm) == :disarmed
    end
  end

  describe "pubsub integration" do
    alias Kinetix.StateMachine.Transition

    test "broadcasts state transitions during lifecycle" do
      start_supervised!(SimpleArm)

      Kinetix.PubSub.subscribe(SimpleArm, [:state_machine])

      {:ok, task} = SimpleArm.arm()
      Task.await(task)

      assert_receive {:kinetix, [:state_machine],
                      %Kinetix.Message{payload: %Transition{from: :disarmed, to: :executing}}}

      assert_receive {:kinetix, [:state_machine],
                      %Kinetix.Message{payload: %Transition{from: :executing, to: :idle}}}
    end

    test "disarm broadcasts transition to disarmed" do
      start_supervised!(SimpleArm)

      {:ok, task} = SimpleArm.arm()
      Task.await(task)

      Kinetix.PubSub.subscribe(SimpleArm, [:state_machine])

      {:ok, task} = SimpleArm.disarm()
      Task.await(task)

      assert_receive {:kinetix, [:state_machine],
                      %Kinetix.Message{payload: %Transition{from: :idle, to: :executing}}}

      assert_receive {:kinetix, [:state_machine],
                      %Kinetix.Message{payload: %Transition{from: :executing, to: :disarmed}}}
    end
  end
end
