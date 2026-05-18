# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.CommandRoutingTest do
  @moduledoc """
  Tests for arm/disarm command routing — the bridge between
  `BB.Safety.arm/1` / `BB.Safety.disarm/2` and DSL-flagged user commands.

  These tests must run synchronously because they interact with the global
  `BB.Safety.Controller` GenServer.
  """
  use ExUnit.Case, async: false

  alias BB.Robot.Runtime

  defmodule HomeAndArmHandler do
    @moduledoc false
    use BB.Command

    alias BB.Safety.Controller

    @impl BB.Command
    def handle_command(_goal, context, state) do
      # Side-effect to verify the handler actually ran.
      :ets.insert(:command_routing_calls, {{context.robot_module, :pre_arm}, true})

      case Controller.arm(context.robot_module) do
        :ok ->
          {:stop, :normal, %{state | result: {:ok, :armed_after_homing}, next_state: :idle}}

        {:error, reason} ->
          {:stop, :normal, %{state | result: {:error, reason}}}
      end
    end

    @impl BB.Command
    def result(%{result: {:ok, value}, next_state: next_state}) do
      {:ok, value, next_state: next_state}
    end

    def result(%{result: result}), do: result
  end

  defmodule SoftDisarmHandler do
    @moduledoc false
    use BB.Command

    alias BB.Safety.Controller

    @impl BB.Command
    def handle_command(_goal, context, state) do
      :ets.insert(:command_routing_calls, {{context.robot_module, :pre_disarm}, true})

      case Controller.disarm(context.robot_module) do
        :ok ->
          {:stop, :normal,
           %{state | result: {:ok, :disarmed_after_motion}, next_state: :disarmed}}

        {:error, reason} ->
          {:stop, :normal, %{state | result: {:error, reason}}}
      end
    end

    @impl BB.Command
    def result(%{result: {:ok, value}, next_state: next_state}) do
      {:ok, value, next_state: next_state}
    end

    def result(%{result: result}), do: result
  end

  defmodule FailingArmHandler do
    @moduledoc false
    use BB.Command

    @impl BB.Command
    def handle_command(_goal, _context, state) do
      # Fails before reaching the safety controller — the safety state stays
      # `:disarmed` and the caller of `BB.Safety.arm/1` should see this error.
      {:stop, :normal, %{state | result: {:error, :pre_arm_check_failed}}}
    end

    @impl BB.Command
    def result(%{result: result}), do: result
  end

  defmodule FailingDisarmHandler do
    @moduledoc false
    use BB.Command

    @impl BB.Command
    def handle_command(_goal, _context, state) do
      # Fails before reaching the safety controller — the safety state stays
      # `:armed`. The routing layer should escalate the robot to `:error`.
      {:stop, :normal, %{state | result: {:error, :motion_to_home_failed}}}
    end

    @impl BB.Command
    def result(%{result: result}), do: result
  end

  defmodule CustomArmRobot do
    @moduledoc false
    use BB
    import BB.Unit

    commands do
      command :home_and_arm do
        handler BB.Safety.CommandRoutingTest.HomeAndArmHandler
        arm true
        allowed_states [:disarmed]
      end

      command :disarm do
        handler BB.Command.Disarm
        allowed_states [:idle]
      end
    end

    topology do
      link :base do
        joint :j1 do
          type :revolute
          actuator :servo, BB.Test.MockActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule CustomDisarmRobot do
    @moduledoc false
    use BB
    import BB.Unit

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :soft_disarm do
        handler BB.Safety.CommandRoutingTest.SoftDisarmHandler
        disarm true
        allowed_states [:idle]
      end
    end

    topology do
      link :base do
        joint :j1 do
          type :revolute
          actuator :servo, BB.Test.MockActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule FailingArmRobot do
    @moduledoc false
    use BB
    import BB.Unit

    commands do
      command :buggy_arm do
        handler BB.Safety.CommandRoutingTest.FailingArmHandler
        arm true
        allowed_states [:disarmed]
      end
    end

    topology do
      link :base do
        joint :j1 do
          type :revolute
          actuator :servo, BB.Test.MockActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule FailingDisarmRobot do
    @moduledoc false
    use BB
    import BB.Unit

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :buggy_disarm do
        handler BB.Safety.CommandRoutingTest.FailingDisarmHandler
        disarm true
        allowed_states [:idle]
      end
    end

    topology do
      link :base do
        joint :j1 do
          type :revolute
          actuator :servo, BB.Test.MockActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  # Robot with no commands at all — exercises the fall-through path.
  defmodule BareRobot do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :j1 do
          type :revolute
          actuator :servo, BB.Test.MockActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  # Robot using the built-in BB.Command.Arm / Disarm — exercises implicit
  # flag handling.
  defmodule BuiltInRobot do
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
    end

    topology do
      link :base do
        joint :j1 do
          type :revolute
          actuator :servo, BB.Test.MockActuator

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  setup do
    case :ets.whereis(:command_routing_calls) do
      :undefined ->
        :ets.new(:command_routing_calls, [:named_table, :public, :set])

      _ ->
        :ets.delete_all_objects(:command_routing_calls)
    end

    :ok
  end

  describe "implicit flags on built-in handlers" do
    test "BB.Command.Arm is the implicit arm-flagged command" do
      assert BuiltInRobot.__bb_arm_command__() == :arm
    end

    test "BB.Command.Disarm is the implicit disarm-flagged command" do
      assert BuiltInRobot.__bb_disarm_command__() == :disarm
    end

    test "robots without an arm command have no flagged command" do
      assert BareRobot.__bb_arm_command__() == nil
      assert BareRobot.__bb_disarm_command__() == nil
    end
  end

  describe "fall-through when no flagged command" do
    test "BB.Safety.arm/1 flips state directly when no arm command" do
      start_supervised!(BareRobot)
      assert BB.Safety.state(BareRobot) == :disarmed

      assert :ok = BB.Safety.arm(BareRobot)
      assert BB.Safety.state(BareRobot) == :armed
    end

    test "BB.Safety.disarm/2 flips state directly when no disarm command" do
      start_supervised!(BareRobot)
      :ok = BB.Safety.arm(BareRobot)

      assert :ok = BB.Safety.disarm(BareRobot)
      assert BB.Safety.state(BareRobot) == :disarmed
    end
  end

  describe "routing for built-in arm/disarm" do
    test "BB.Safety.arm/1 routes through implicit arm command" do
      start_supervised!(BuiltInRobot)
      BB.PubSub.subscribe(BuiltInRobot, [:command])

      assert :ok = BB.Safety.arm(BuiltInRobot)
      assert BB.Safety.state(BuiltInRobot) == :armed

      # Confirm a :command event was published — proof that routing went
      # through the command pipeline rather than a direct controller flip.
      assert_receive {:bb, [:command | _], %BB.Message{}}, 1_000
    end

    test "BB.Safety.disarm/2 routes through implicit disarm command" do
      start_supervised!(BuiltInRobot)
      :ok = BB.Safety.arm(BuiltInRobot)
      BB.PubSub.subscribe(BuiltInRobot, [:command])

      assert :ok = BB.Safety.disarm(BuiltInRobot)
      assert BB.Safety.state(BuiltInRobot) == :disarmed

      assert_receive {:bb, [:command | _], %BB.Message{}}, 1_000
    end
  end

  describe "routing through user-defined arm command" do
    test "BB.Safety.arm/1 dispatches user-defined arm command" do
      start_supervised!(CustomArmRobot)

      assert :ok = BB.Safety.arm(CustomArmRobot)
      assert BB.Safety.state(CustomArmRobot) == :armed

      assert :ets.lookup(:command_routing_calls, {CustomArmRobot, :pre_arm}) == [
               {{CustomArmRobot, :pre_arm}, true}
             ]
    end

    test "calling user-defined arm command directly produces the same result" do
      start_supervised!(CustomArmRobot)

      {:ok, cmd} = CustomArmRobot.home_and_arm()
      assert {:ok, :armed_after_homing, _} = BB.Command.await(cmd)
      assert BB.Safety.state(CustomArmRobot) == :armed
    end
  end

  describe "routing through user-defined disarm command" do
    test "BB.Safety.disarm/2 dispatches user-defined disarm command" do
      start_supervised!(CustomDisarmRobot)
      :ok = BB.Safety.arm(CustomDisarmRobot)

      assert :ok = BB.Safety.disarm(CustomDisarmRobot)
      assert BB.Safety.state(CustomDisarmRobot) == :disarmed

      assert :ets.lookup(:command_routing_calls, {CustomDisarmRobot, :pre_disarm}) == [
               {{CustomDisarmRobot, :pre_disarm}, true}
             ]
    end
  end

  describe "failure semantics" do
    test "arm command failure leaves state at :disarmed" do
      start_supervised!(FailingArmRobot)

      assert {:error, :pre_arm_check_failed} = BB.Safety.arm(FailingArmRobot)
      assert BB.Safety.state(FailingArmRobot) == :disarmed
    end

    test "disarm command failure before flipping state escalates to :error" do
      start_supervised!(FailingDisarmRobot)
      :ok = BB.Safety.arm(FailingDisarmRobot)
      assert BB.Safety.state(FailingDisarmRobot) == :armed

      assert {:error, {:disarm_command_failed, :motion_to_home_failed}} =
               BB.Safety.disarm(FailingDisarmRobot)

      assert BB.Safety.state(FailingDisarmRobot) == :error
      assert BB.Safety.in_error?(FailingDisarmRobot)
    end

    test "force_disarm/1 recovers from disarm-command-failed escalation" do
      start_supervised!(FailingDisarmRobot)
      :ok = BB.Safety.arm(FailingDisarmRobot)
      {:error, {:disarm_command_failed, _}} = BB.Safety.disarm(FailingDisarmRobot)

      assert :ok = BB.Safety.force_disarm(FailingDisarmRobot)
      assert BB.Safety.state(FailingDisarmRobot) == :disarmed
    end
  end

  describe "re-entrancy" do
    test "BB.Command.Arm calls Controller.arm directly without looping" do
      start_supervised!(BuiltInRobot)

      # Should complete without infinite recursion / timeout.
      task = Task.async(fn -> BB.Safety.arm(BuiltInRobot) end)
      assert :ok = Task.await(task, 2_000)

      assert BB.Safety.state(BuiltInRobot) == :armed
    end

    test "operational state matches initial_state after routed arm" do
      start_supervised!(BuiltInRobot)

      :ok = BB.Safety.arm(BuiltInRobot)
      # BuiltInRobot uses the default initial_state of :idle
      assert Runtime.operational_state(BuiltInRobot) == :idle
    end
  end
end
