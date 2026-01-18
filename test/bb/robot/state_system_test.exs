# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.StateSystemTest do
  use ExUnit.Case, async: true

  alias BB.Dsl.Info
  alias BB.Robot.Runtime

  describe "states DSL" do
    defmodule RobotWithCustomStates do
      use BB

      states do
        initial_state :idle

        state :recording do
          doc "Recording sensor data"
        end

        state :processing do
          doc "Processing recorded data"
        end
      end

      commands do
        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle, :recording, :processing]
          cancel :*
        end
      end

      topology do
        link :base_link
      end
    end

    test "states/1 returns all defined states including :idle" do
      states = Info.states(RobotWithCustomStates)

      state_names = Enum.map(states, & &1.name)
      assert :idle in state_names
      assert :recording in state_names
      assert :processing in state_names
    end

    test "state_names/1 returns list of state names" do
      names = Info.state_names(RobotWithCustomStates)

      assert :idle in names
      assert :recording in names
      assert :processing in names
    end

    test "initial_state/1 returns configured initial state" do
      assert Info.initial_state(RobotWithCustomStates) == :idle
    end

    test "state entity has doc field" do
      states = Info.states(RobotWithCustomStates)
      recording = Enum.find(states, &(&1.name == :recording))

      assert recording.doc == "Recording sensor data"
    end
  end

  describe "states DSL with custom initial state" do
    defmodule RobotWithCustomInitialState do
      use BB

      states do
        initial_state :standby

        state :standby do
          doc "Waiting for activation"
        end
      end

      commands do
        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:standby]
          cancel :*
        end
      end

      topology do
        link :base_link
      end
    end

    test "initial_state/1 returns custom initial state" do
      assert Info.initial_state(RobotWithCustomInitialState) == :standby
    end

    test "robot starts in custom initial state after arming" do
      start_supervised!(RobotWithCustomInitialState)

      :ok = BB.Safety.arm(RobotWithCustomInitialState)

      assert Runtime.operational_state(RobotWithCustomInitialState) == :standby
    end
  end

  describe "categories DSL" do
    defmodule RobotWithCategories do
      use BB

      commands do
        category :motion do
          doc "Physical movement commands"
          concurrency_limit 1
        end

        category :sensing do
          doc "Sensor commands"
          concurrency_limit 2
        end

        category :auxiliary do
          doc "LEDs, sounds, etc"
          concurrency_limit 3
        end

        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle]
          cancel :*
        end

        command :move do
          handler BB.Test.AsyncCommand
          category :motion
          allowed_states [:idle]
          cancel [:motion]
        end

        command :scan do
          handler BB.Test.AsyncCommand
          category :sensing
          allowed_states [:idle]
          cancel [:sensing]
        end

        command :blink do
          handler BB.Test.AsyncCommand
          category :auxiliary
          allowed_states [:idle]
          cancel [:auxiliary]
        end

        command :default_cmd do
          handler BB.Test.AsyncCommand
          allowed_states [:idle]
          cancel [:default]
        end
      end

      topology do
        link :base_link
      end
    end

    test "categories/1 returns all defined categories including :default" do
      categories = Info.categories(RobotWithCategories)

      category_names = Enum.map(categories, & &1.name)
      assert :default in category_names
      assert :motion in category_names
      assert :sensing in category_names
      assert :auxiliary in category_names
    end

    test "category_limits/1 returns concurrency limits" do
      limits = Info.category_limits(RobotWithCategories)

      assert limits[:default] == 1
      assert limits[:motion] == 1
      assert limits[:sensing] == 2
      assert limits[:auxiliary] == 3
    end

    test "category entity has doc field" do
      categories = Info.categories(RobotWithCategories)
      motion = Enum.find(categories, &(&1.name == :motion))

      assert motion.doc == "Physical movement commands"
    end
  end

  describe "BB.Command.SetState" do
    defmodule RobotWithSetState do
      use BB

      states do
        state :recording
        state :playback
      end

      commands do
        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle, :recording, :playback]
          cancel :*
        end

        command :enter_recording do
          handler {BB.Command.SetState, to: :recording}
          allowed_states [:idle]
        end

        command :exit_recording do
          handler {BB.Command.SetState, to: :idle}
          allowed_states [:recording]
        end

        command :enter_playback do
          handler {BB.Command.SetState, to: :playback}
          allowed_states [:idle, :recording]
        end
      end

      topology do
        link :base_link
      end
    end

    test "SetState transitions to target state" do
      start_supervised!(RobotWithSetState)

      :ok = BB.Safety.arm(RobotWithSetState)
      assert Runtime.operational_state(RobotWithSetState) == :idle

      {:ok, cmd} = Runtime.execute(RobotWithSetState, :enter_recording, %{})
      assert {:ok, :recording, _opts} = BB.Command.await(cmd)

      assert Runtime.operational_state(RobotWithSetState) == :recording
    end

    test "SetState respects allowed_states" do
      start_supervised!(RobotWithSetState)

      :ok = BB.Safety.arm(RobotWithSetState)

      # Try to exit_recording when in :idle - should fail
      assert {:error, %BB.Error.State.NotAllowed{current_state: :idle}} =
               Runtime.execute(RobotWithSetState, :exit_recording, %{})
    end

    test "SetState can chain transitions" do
      start_supervised!(RobotWithSetState)

      :ok = BB.Safety.arm(RobotWithSetState)

      # idle -> recording
      {:ok, cmd1} = Runtime.execute(RobotWithSetState, :enter_recording, %{})
      assert {:ok, :recording, _opts1} = BB.Command.await(cmd1)

      # recording -> playback
      {:ok, cmd2} = Runtime.execute(RobotWithSetState, :enter_playback, %{})
      assert {:ok, :playback, _opts2} = BB.Command.await(cmd2)

      assert Runtime.operational_state(RobotWithSetState) == :playback
    end
  end

  describe "introspection APIs" do
    defmodule RobotForIntrospection do
      use BB

      commands do
        category :motion do
          concurrency_limit 1
        end

        category :sensing do
          concurrency_limit 2
        end

        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle]
          cancel :*
        end

        command :async_motion do
          handler BB.Test.AsyncCommand
          category :motion
          allowed_states [:idle]
          cancel [:motion]
        end

        command :async_sensing do
          handler BB.Test.AsyncCommand
          category :sensing
          allowed_states [:idle]
          cancel [:sensing]
        end
      end

      topology do
        link :base_link
      end
    end

    test "operational_state/1 returns current operational state" do
      start_supervised!(RobotForIntrospection)

      :ok = BB.Safety.arm(RobotForIntrospection)

      assert Runtime.operational_state(RobotForIntrospection) == :idle
    end

    test "executing?/1 returns false when no commands running" do
      start_supervised!(RobotForIntrospection)

      :ok = BB.Safety.arm(RobotForIntrospection)

      refute Runtime.executing?(RobotForIntrospection)
    end

    test "executing?/1 returns true when commands running" do
      start_supervised!(RobotForIntrospection)

      :ok = BB.Safety.arm(RobotForIntrospection)

      {:ok, cmd} = Runtime.execute(RobotForIntrospection, :async_motion, %{notify: self()})
      assert_receive {:executing, ^cmd}, 500

      assert Runtime.executing?(RobotForIntrospection)

      send(cmd, :complete)
    end

    test "executing?/2 checks specific category" do
      start_supervised!(RobotForIntrospection)

      :ok = BB.Safety.arm(RobotForIntrospection)

      {:ok, cmd} = Runtime.execute(RobotForIntrospection, :async_motion, %{notify: self()})
      assert_receive {:executing, ^cmd}, 500

      assert Runtime.executing?(RobotForIntrospection, :motion)
      refute Runtime.executing?(RobotForIntrospection, :sensing)

      send(cmd, :complete)
    end

    test "executing_commands/1 returns list of running commands" do
      start_supervised!(RobotForIntrospection)

      :ok = BB.Safety.arm(RobotForIntrospection)

      {:ok, cmd} = Runtime.execute(RobotForIntrospection, :async_motion, %{notify: self()})
      assert_receive {:executing, ^cmd}, 500

      commands = Runtime.executing_commands(RobotForIntrospection)
      assert length(commands) == 1

      [command_info] = commands
      assert command_info.name == :async_motion
      assert command_info.category == :motion
      assert command_info.pid == cmd

      send(cmd, :complete)
    end

    test "category_availability/1 returns current counts and limits" do
      start_supervised!(RobotForIntrospection)

      :ok = BB.Safety.arm(RobotForIntrospection)

      availability = Runtime.category_availability(RobotForIntrospection)
      assert availability[:motion] == {0, 1}
      assert availability[:sensing] == {0, 2}
      assert availability[:default] == {0, 1}

      {:ok, cmd} = Runtime.execute(RobotForIntrospection, :async_motion, %{notify: self()})
      assert_receive {:executing, ^cmd}, 500

      availability = Runtime.category_availability(RobotForIntrospection)
      assert availability[:motion] == {1, 1}
      assert availability[:sensing] == {0, 2}

      send(cmd, :complete)
    end
  end

  describe "category concurrency" do
    defmodule RobotWithConcurrency do
      use BB

      states do
        state :active do
          doc "Active mode for testing category concurrency"
        end
      end

      commands do
        category :parallel do
          concurrency_limit 3
        end

        category :limited do
          concurrency_limit 1
        end

        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle, :active]
          cancel :*
        end

        command :enter_active do
          handler {BB.Command.SetState, to: :active}
          allowed_states [:idle]
        end

        command :parallel_cmd do
          handler BB.Test.AsyncCommand
          category :parallel
          allowed_states [:idle]
          # No cancel - allows concurrent execution up to category limit
        end

        command :exclusive_cmd do
          handler BB.Test.AsyncCommand
          allowed_states [:idle]
          cancel [:default]
        end

        command :limited_cmd do
          handler BB.Test.AsyncCommand
          category :limited
          allowed_states [:active]
          cancel [:limited]
        end

        command :limited_no_preempt do
          handler BB.Test.AsyncCommand
          category :limited
          # No cancel, so cannot preempt
          allowed_states [:active]
        end

        command :non_preemptable_cmd do
          handler BB.Test.AsyncCommand
          allowed_states [:idle]
          # No cancel - will error if :default category is full
        end
      end

      topology do
        link :base_link
      end
    end

    test "multiple commands can run in same category up to limit" do
      start_supervised!(RobotWithConcurrency)

      :ok = BB.Safety.arm(RobotWithConcurrency)

      # Start 3 parallel commands (at the limit)
      {:ok, cmd1} = Runtime.execute(RobotWithConcurrency, :parallel_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd1}, 500

      {:ok, cmd2} = Runtime.execute(RobotWithConcurrency, :parallel_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd2}, 500

      {:ok, cmd3} = Runtime.execute(RobotWithConcurrency, :parallel_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd3}, 500

      # All 3 should be running
      assert length(Runtime.executing_commands(RobotWithConcurrency)) == 3

      # Clean up
      send(cmd1, :complete)
      send(cmd2, :complete)
      send(cmd3, :complete)
    end

    test "category returns error when at capacity without preemption" do
      start_supervised!(RobotWithConcurrency)

      :ok = BB.Safety.arm(RobotWithConcurrency)

      # Start non_preemptable_cmd which uses :default category with limit 1
      {:ok, cmd1} =
        Runtime.execute(RobotWithConcurrency, :non_preemptable_cmd, %{notify: self()})

      assert_receive {:executing, ^cmd1}, 500

      # Second non_preemptable_cmd should fail because:
      # - :default category is at capacity (1/1)
      # - :non_preemptable_cmd doesn't have :executing in allowed_states, so can't preempt
      assert {:error, %BB.Error.Category.Full{category: :default, limit: 1, current: 1}} =
               Runtime.execute(RobotWithConcurrency, :non_preemptable_cmd, %{notify: self()})

      send(cmd1, :complete)
    end

    test "preemption works when category full and :executing in allowed_states" do
      start_supervised!(RobotWithConcurrency)

      :ok = BB.Safety.arm(RobotWithConcurrency)

      # Start first command
      {:ok, cmd1} = Runtime.execute(RobotWithConcurrency, :exclusive_cmd, %{notify: self()})
      ref = Process.monitor(cmd1)
      assert_receive {:executing, ^cmd1}, 500

      # Second command should preempt (since :executing in allowed_states)
      {:ok, cmd2} = Runtime.execute(RobotWithConcurrency, :exclusive_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd2}, 500

      # First command should be terminated
      assert_receive {:DOWN, ^ref, :process, ^cmd1, _reason}, 500
      assert {:error, :cancelled} = BB.Command.await(cmd1)

      send(cmd2, :complete)
    end

    test "commands in different categories can run concurrently" do
      start_supervised!(RobotWithConcurrency)

      :ok = BB.Safety.arm(RobotWithConcurrency)

      # Start command in :parallel category
      {:ok, cmd1} = Runtime.execute(RobotWithConcurrency, :parallel_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd1}, 500

      # Start command in :default category - should work
      {:ok, cmd2} = Runtime.execute(RobotWithConcurrency, :exclusive_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd2}, 500

      # Both should be running
      commands = Runtime.executing_commands(RobotWithConcurrency)
      assert length(commands) == 2

      categories = Enum.map(commands, & &1.category)
      assert :parallel in categories
      assert :default in categories

      send(cmd1, :complete)
      send(cmd2, :complete)
    end
  end

  describe "state/1 backwards compatibility" do
    defmodule RobotForBackwardsCompat do
      use BB

      states do
        state :recording
      end

      commands do
        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle, :recording]
          cancel :*
        end

        command :async_cmd do
          handler BB.Test.AsyncCommand
          allowed_states [:idle]
          cancel [:default]
        end

        command :enter_recording do
          handler {BB.Command.SetState, to: :recording}
          allowed_states [:idle]
        end
      end

      topology do
        link :base_link
      end
    end

    test "state/1 returns :executing when in :idle with commands running" do
      start_supervised!(RobotForBackwardsCompat)

      :ok = BB.Safety.arm(RobotForBackwardsCompat)

      assert Runtime.state(RobotForBackwardsCompat) == :idle
      assert Runtime.operational_state(RobotForBackwardsCompat) == :idle

      {:ok, cmd} = Runtime.execute(RobotForBackwardsCompat, :async_cmd, %{notify: self()})
      assert_receive {:executing, ^cmd}, 500

      # state/1 returns :executing for backwards compat
      assert Runtime.state(RobotForBackwardsCompat) == :executing
      # operational_state/1 returns actual state
      assert Runtime.operational_state(RobotForBackwardsCompat) == :idle

      send(cmd, :complete)
    end

    test "state/1 returns custom state even with commands running" do
      start_supervised!(RobotForBackwardsCompat)

      :ok = BB.Safety.arm(RobotForBackwardsCompat)

      # Enter recording state
      {:ok, cmd1} = Runtime.execute(RobotForBackwardsCompat, :enter_recording, %{})
      assert {:ok, :recording, _opts} = BB.Command.await(cmd1)

      assert Runtime.state(RobotForBackwardsCompat) == :recording
      assert Runtime.operational_state(RobotForBackwardsCompat) == :recording
    end
  end

  describe "BB.Command.transition_state/2" do
    defmodule TransitionCommand do
      use BB.Command

      @impl BB.Command
      def handle_command(goal, context, state) do
        send(goal.notify, {:executing, self()})

        state =
          state
          |> Map.put(:context, context)
          |> Map.put(:current_idx, 0)

        case goal.transitions do
          [first | _] ->
            :ok = BB.Command.transition_state(context, first)
            send(goal.notify, {:transitioned_to, first})

          [] ->
            :ok
        end

        {:noreply, state}
      end

      @impl BB.Command
      def handle_info(:next_transition, state) do
        next_idx = state.current_idx + 1
        transitions = state.goal.transitions

        case Enum.at(transitions, next_idx) do
          nil ->
            {:noreply, state}

          target ->
            :ok = BB.Command.transition_state(state.context, target)
            send(state.goal.notify, {:transitioned_to, target})
            {:noreply, Map.put(state, :current_idx, next_idx)}
        end
      end

      def handle_info(:complete, state) do
        {:stop, :normal, Map.put(state, :result, {:ok, :done})}
      end

      def handle_info(_msg, state), do: {:noreply, state}

      @impl BB.Command
      def result(%{result: nil}), do: {:error, :cancelled}
      def result(%{result: result}), do: result
    end

    defmodule RobotWithTransitions do
      use BB

      states do
        state :phase1
        state :phase2
        state :phase3
      end

      commands do
        command :arm do
          handler BB.Command.Arm
          allowed_states [:disarmed]
        end

        command :disarm do
          handler BB.Command.Disarm
          allowed_states [:idle, :phase1, :phase2, :phase3]
          cancel :*
        end

        command :multi_phase do
          handler BB.Robot.StateSystemTest.TransitionCommand
          allowed_states [:idle]
        end
      end

      topology do
        link :base_link
      end
    end

    test "command can transition state mid-execution" do
      start_supervised!(RobotWithTransitions)

      :ok = BB.Safety.arm(RobotWithTransitions)

      {:ok, cmd} =
        Runtime.execute(RobotWithTransitions, :multi_phase, %{
          notify: self(),
          transitions: [:phase1]
        })

      assert_receive {:executing, ^cmd}, 500
      assert_receive {:transitioned_to, :phase1}, 500

      assert Runtime.operational_state(RobotWithTransitions) == :phase1

      send(cmd, :complete)
    end

    test "command can transition through multiple states" do
      start_supervised!(RobotWithTransitions)

      :ok = BB.Safety.arm(RobotWithTransitions)

      {:ok, cmd} =
        Runtime.execute(RobotWithTransitions, :multi_phase, %{
          notify: self(),
          transitions: [:phase1, :phase2, :phase3]
        })

      assert_receive {:executing, ^cmd}, 500
      assert_receive {:transitioned_to, :phase1}, 500
      assert Runtime.operational_state(RobotWithTransitions) == :phase1

      send(cmd, :next_transition)
      assert_receive {:transitioned_to, :phase2}, 500
      assert Runtime.operational_state(RobotWithTransitions) == :phase2

      send(cmd, :next_transition)
      assert_receive {:transitioned_to, :phase3}, 500
      assert Runtime.operational_state(RobotWithTransitions) == :phase3

      send(cmd, :complete)
    end
  end

  describe "compile-time validation" do
    import ExUnit.CaptureIO

    test "verifier warns about undefined state in allowed_states" do
      output =
        capture_io(:stderr, fn ->
          Code.compile_string("""
          defmodule InvalidStateRef#{System.unique_integer([:positive])} do
            use BB

            commands do
              command :arm do
                handler BB.Command.Arm
                allowed_states [:disarmed]
              end

              command :bad_cmd do
                handler BB.Test.ImmediateSuccessCommand
                allowed_states [:nonexistent_state]
              end
            end

            topology do
              link :base_link
            end
          end
          """)
        end)

      assert output =~ "references undefined states"
      assert output =~ ":nonexistent_state"
    end

    test "verifier warns about undefined state in SetState target" do
      output =
        capture_io(:stderr, fn ->
          Code.compile_string("""
          defmodule InvalidSetStateTarget#{System.unique_integer([:positive])} do
            use BB

            commands do
              command :arm do
                handler BB.Command.Arm
                allowed_states [:disarmed]
              end

              command :bad_transition do
                handler {BB.Command.SetState, to: :nonexistent}
                allowed_states [:idle]
              end
            end

            topology do
              link :base_link
            end
          end
          """)
        end)

      assert output =~ "targets undefined state"
      assert output =~ ":nonexistent"
    end

    test "verifier warns about undefined initial_state" do
      output =
        capture_io(:stderr, fn ->
          Code.compile_string("""
          defmodule InvalidInitialState#{System.unique_integer([:positive])} do
            use BB

            states do
              initial_state :nonexistent
            end

            commands do
              command :arm do
                handler BB.Command.Arm
                allowed_states [:disarmed]
              end
            end

            topology do
              link :base_link
            end
          end
          """)
        end)

      assert output =~ "Invalid initial_state"
      assert output =~ ":nonexistent"
    end

    test "verifier warns about undefined category reference" do
      output =
        capture_io(:stderr, fn ->
          Code.compile_string("""
          defmodule InvalidCategoryRef#{System.unique_integer([:positive])} do
            use BB

            commands do
              command :arm do
                handler BB.Command.Arm
                allowed_states [:disarmed]
              end

              command :bad_cmd do
                handler BB.Test.ImmediateSuccessCommand
                category :nonexistent_category
                allowed_states [:idle]
              end
            end

            topology do
              link :base_link
            end
          end
          """)
        end)

      assert output =~ "references undefined category"
      assert output =~ ":nonexistent_category"
    end
  end
end
