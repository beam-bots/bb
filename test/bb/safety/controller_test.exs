# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.ControllerTest do
  @moduledoc """
  Tests for BB.Safety.Controller error state handling and force_disarm/1.

  These tests must run synchronously because they interact with the global
  BB.Safety.Controller GenServer and need robots in a clean state.
  """
  use ExUnit.Case, async: false

  alias __MODULE__.Hardware
  alias BB.StateMachine.Transition

  defmodule RobotWithFailingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :failing, {BB.Test.FailingActuator, fail_mode: :error}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule RobotWithRaisingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :failing, {BB.Test.FailingActuator, fail_mode: :raise}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule RobotWithThrowingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :failing, {BB.Test.FailingActuator, fail_mode: :throw}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule RobotWithHangingActuator do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
          type :revolute
          actuator :hanging, {BB.Test.FailingActuator, fail_mode: :hang}

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :child
        end
      end
    end
  end

  defmodule HardwareController do
    @moduledoc false
    use BB.Controller, options_schema: [hardware: [type: :atom, required: true]]

    # `:lost` crashes after `init/1` returns, so every restart uses up budget
    # straight away; `:missing` fails `init/1` itself, so the start fails.
    @impl BB.Controller
    def init(opts) do
      case Agent.get(opts[:hardware], & &1) do
        :present -> {:ok, %{}}
        :lost -> {:ok, %{}, {:continue, :lose_hardware}}
        :missing -> {:stop, :hardware_missing}
      end
    end

    @impl BB.Controller
    def handle_continue(:lose_hardware, state), do: {:stop, :hardware_lost, state}
  end

  defmodule RobotWithHardwareController do
    @moduledoc false
    use BB

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end
    end

    controllers do
      controller :hardware,
                 {BB.Safety.ControllerTest.HardwareController, hardware: Hardware}
    end

    topology do
      link :base do
      end
    end
  end

  describe "error state on disarm callback failure" do
    test "disarm transitions to error state when callback returns error" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert BB.Safety.state(RobotWithFailingActuator) == :armed

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert BB.Safety.state(RobotWithFailingActuator) == :error
      assert BB.Safety.in_error?(RobotWithFailingActuator) == true
      assert length(failures) == 1
      assert {[:base, :joint1, :failing], {:returned_error, :hardware_failure}} in failures
    end

    test "disarm transitions to error state when callback raises" do
      start_supervised!(RobotWithRaisingActuator)

      :ok = BB.Safety.arm(RobotWithRaisingActuator)

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithRaisingActuator)

      assert BB.Safety.state(RobotWithRaisingActuator) == :error
      assert length(failures) == 1

      assert {[:base, :joint1, :failing], {:exception, "Hardware communication failed"}} in failures
    end

    test "disarm transitions to error state when callback throws" do
      start_supervised!(RobotWithThrowingActuator)

      :ok = BB.Safety.arm(RobotWithThrowingActuator)

      {:error, {:disarm_failed, failures}} = BB.Safety.disarm(RobotWithThrowingActuator)

      assert BB.Safety.state(RobotWithThrowingActuator) == :error
      assert length(failures) == 1
      assert {[:base, :joint1, :failing], {:throw, :hardware_timeout}} in failures
    end
  end

  describe "cannot arm in error state" do
    test "arm returns error when robot is in error state" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert {:error, :in_error} = BB.Safety.arm(RobotWithFailingActuator)
    end
  end

  describe "force_disarm/1" do
    test "resets error state to disarmed" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      assert BB.Safety.state(RobotWithFailingActuator) == :error

      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert BB.Safety.state(RobotWithFailingActuator) == :disarmed
      assert BB.Safety.in_error?(RobotWithFailingActuator) == false
    end

    test "allows arming after force_disarm" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert BB.Safety.armed?(RobotWithFailingActuator) == true
    end

    test "leaves a running topology alone" do
      start_supervised!(RobotWithFailingActuator)
      topology = BB.Process.whereis(RobotWithFailingActuator, BB.TopologySupervisor)
      actuator = BB.Process.whereis(RobotWithFailingActuator, :failing)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert BB.Process.whereis(RobotWithFailingActuator, BB.TopologySupervisor) == topology
      assert BB.Process.whereis(RobotWithFailingActuator, :failing) == actuator
    end

    test "returns error when not in error state" do
      start_supervised!(RobotWithFailingActuator)

      assert {:error, :not_in_error} = BB.Safety.force_disarm(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert {:error, :not_in_error} = BB.Safety.force_disarm(RobotWithFailingActuator)
    end
  end

  describe "pubsub transitions for error state" do
    test "publishes error transition when disarm fails" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)

      BB.PubSub.subscribe(RobotWithFailingActuator, [:state_machine])

      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :disarming, to: :error}}}
    end

    test "publishes disarmed transition on force_disarm" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      BB.PubSub.subscribe(RobotWithFailingActuator, [:state_machine])

      :ok = BB.Safety.force_disarm(RobotWithFailingActuator)

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :error, to: :disarmed}}}
    end
  end

  describe "disarming state" do
    test "publishes disarming transition before callbacks run" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      BB.PubSub.subscribe(RobotWithFailingActuator, [:state_machine])

      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert_receive {:bb, [:state_machine],
                      %BB.Message{payload: %Transition{from: :armed, to: :disarming}}}
    end

    test "disarming?/1 returns true while disarming" do
      start_supervised!(RobotWithFailingActuator)

      assert BB.Safety.disarming?(RobotWithFailingActuator) == false

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      assert BB.Safety.disarming?(RobotWithFailingActuator) == false

      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)
      assert BB.Safety.disarming?(RobotWithFailingActuator) == false
    end

    test "double disarm returns error" do
      start_supervised!(RobotWithFailingActuator)

      :ok = BB.Safety.arm(RobotWithFailingActuator)
      {:error, {:disarm_failed, _}} = BB.Safety.disarm(RobotWithFailingActuator)

      assert {:error, :already_in_error} = BB.Safety.disarm(RobotWithFailingActuator)
    end
  end

  describe "timeout handling" do
    test "a callback that never returns times out and transitions to error state" do
      start_supervised!(RobotWithHangingActuator)

      :ok = BB.Safety.arm(RobotWithHangingActuator)

      {:error, {:disarm_failed, failures}} =
        BB.Safety.disarm(RobotWithHangingActuator, timeout: 50)

      assert BB.Safety.state(RobotWithHangingActuator) == :error
      assert length(failures) == 1
      assert {:unknown, {:timeout, 50}} in failures
    end
  end

  defmodule RobotForReporting do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base do
        joint :joint1 do
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

  describe "report_error/3" do
    test "publishes a hardware error message" do
      start_supervised!(RobotForReporting)

      BB.PubSub.subscribe(RobotForReporting, [:safety, :error])

      BB.Safety.report_error(
        RobotForReporting,
        [:controller, :servo_1],
        {:hardware_error, 0x04}
      )

      assert_receive {:bb, [:safety, :error],
                      %BB.Message{
                        payload: %BB.Safety.HardwareError{
                          path: [:controller, :servo_1],
                          error: {:hardware_error, 0x04}
                        }
                      }}
    end

    test "does not disarm an armed robot" do
      start_supervised!(RobotForReporting)
      :ok = BB.Safety.arm(RobotForReporting)
      BB.PubSub.subscribe(RobotForReporting, [:state_machine])

      BB.Safety.report_error(
        RobotForReporting,
        [:controller, :servo_1],
        {:hardware_error, 0x04}
      )

      :sys.get_state(BB.Safety.Controller)

      assert BB.Safety.armed?(RobotForReporting)
      refute_received {:bb, [:state_machine], _}
    end
  end

  describe "topology supervisor failure" do
    test "force-disarms and transitions to :error when topology supervisor stops" do
      Process.flag(:trap_exit, true)

      start_supervised!(RobotForReporting)
      :ok = BB.Safety.arm(RobotForReporting)

      BB.PubSub.subscribe(RobotForReporting, [:state_machine])

      topology_pid =
        BB.Process.whereis(RobotForReporting, BB.TopologySupervisor)

      assert is_pid(topology_pid)

      Process.exit(topology_pid, :kill)

      assert_receive {:bb, [:state_machine], %BB.Message{payload: %Transition{to: :error}}},
                     1_000

      assert BB.Safety.state(RobotForReporting) == :error
    end
  end

  describe "recovering from a stopped topology" do
    setup do
      start_supervised!(%{
        id: Hardware,
        start: {Agent, :start_link, [fn -> :present end, [name: Hardware]]}
      })

      start_supervised!(RobotWithHardwareController)
      BB.PubSub.subscribe(RobotWithHardwareController, [:state_machine])
      :ok
    end

    test "force_disarm/1 restarts the topology so the robot can be armed again" do
      lose_hardware()
      Agent.update(Hardware, fn _ -> :present end)

      assert :ok = BB.Safety.force_disarm(RobotWithHardwareController)

      assert is_pid(BB.Process.whereis(RobotWithHardwareController, BB.TopologySupervisor))
      assert is_pid(BB.Process.whereis(RobotWithHardwareController, :hardware))
      assert BB.Safety.state(RobotWithHardwareController) == :disarmed
      assert :ok = BB.Safety.arm(RobotWithHardwareController)
    end

    test "force_disarm/1 returns to :error when the topology cannot be restarted" do
      lose_hardware()
      Agent.update(Hardware, fn _ -> :missing end)

      assert {:error, {:topology_restart_failed, _reason}} =
               BB.Safety.force_disarm(RobotWithHardwareController)

      assert BB.Safety.state(RobotWithHardwareController) == :error

      assert BB.Process.whereis(RobotWithHardwareController, BB.TopologySupervisor) ==
               :undefined
    end

    test "arming fails while the topology is not running" do
      stop_topology_while_disarmed()

      assert {:error, :topology_not_running} =
               BB.Safety.Controller.arm(RobotWithHardwareController)

      assert {:error, :topology_not_running} = BB.Safety.arm(RobotWithHardwareController)
      assert BB.Safety.state(RobotWithHardwareController) == :disarmed
    end
  end

  defp lose_hardware do
    Agent.update(Hardware, fn _ -> :lost end)

    RobotWithHardwareController
    |> BB.Process.whereis(:hardware)
    |> Process.exit(:kill)

    assert_receive {:bb, [:state_machine], %BB.Message{payload: %Transition{to: :error}}}, 1_000
  end

  defp stop_topology_while_disarmed do
    :ok = Supervisor.terminate_child(RobotWithHardwareController, BB.TopologySupervisor)

    assert_receive {:bb, [:state_machine], %BB.Message{payload: %Transition{to: :error}}}, 1_000

    # Stopping the topology latches :error, and the public force_disarm/1 would
    # restart it. Acknowledge through the controller alone to reach :disarmed
    # with no topology behind it.
    :ok = GenServer.call(BB.Safety.Controller, {:force_disarm, RobotWithHardwareController})
  end
end
