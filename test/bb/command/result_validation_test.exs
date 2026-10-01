# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.ResultValidationTest do
  @moduledoc """
  A handler whose `result/1` breaks its contract is a coding mistake in the
  handler, so the failure has to land there rather than on the robot.
  """
  use ExUnit.Case, async: true

  alias BB.Error.Invalid.CommandResult
  alias BB.Error.State.CommandCrashed
  alias BB.Robot.Runtime

  @moduletag :capture_log

  defmodule Robot do
    @moduledoc false
    use BB

    states do
      state :parked
    end

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :disarm do
        handler BB.Command.Disarm
        allowed_states [:idle, :parked]
      end

      command :bad do
        handler BB.Test.BadResultCommand
        allowed_states [:idle]
      end
    end

    topology do
      link :base
    end
  end

  setup do
    start_supervised!(Robot)
    :ok = BB.Safety.arm(Robot)
    BB.PubSub.subscribe(Robot, [:command])
    %{runtime: GenServer.whereis(Runtime.via(Robot))}
  end

  describe "a result/1 outside its contract" do
    test "is replaced with a structured error naming the handler" do
      assert {:error, %CommandResult{} = error} = run(%{result: :done})

      assert error.command == BB.Test.BadResultCommand
      assert error.value == :done
      assert Exception.message(error) =~ "result/1 returned :done"
    end

    test "leaves the runtime and the robot alone", %{runtime: runtime} do
      assert {:error, %CommandResult{}} = run(%{result: :done})

      assert GenServer.whereis(Runtime.via(Robot)) == runtime
      assert BB.Safety.state(Robot) == :armed
      assert Runtime.operational_state(Robot) == :idle
    end

    test "rejects options that aren't a keyword list" do
      assert {:error, %CommandResult{value: {:ok, :done, [:parked]}}} =
               run(%{result: {:ok, :done, [:parked]}})
    end
  end

  describe "a raising result/1" do
    test "is reported as a command crash" do
      assert {:error, %CommandCrashed{} = error} = run(%{raise: true})
      assert error.command == BB.Test.BadResultCommand
    end
  end

  describe "next_state" do
    test "moves the robot to a declared state" do
      assert {:ok, :done, next_state: :parked} = run(%{result: {:ok, :done, next_state: :parked}})

      assert Runtime.operational_state(Robot) == :parked
    end

    test "accepts :disarmed, which is built in rather than declared" do
      assert {:ok, :disarmed, _} = run(:disarm, %{})

      assert Runtime.operational_state(Robot) == :disarmed
    end

    test "refuses the safety states the safety system owns", %{runtime: runtime} do
      for state <- [:disarming, :error] do
        assert {:ok, :done, next_state: ^state} =
                 run(%{result: {:ok, :done, next_state: state}})

        assert Runtime.operational_state(Robot) == :idle
      end

      assert GenServer.whereis(Runtime.via(Robot)) == runtime
    end

    test "is ignored when it names a state the robot doesn't have", %{runtime: runtime} do
      assert {:ok, :done, next_state: :nowhere} =
               run(%{result: {:ok, :done, next_state: :nowhere}})

      assert GenServer.whereis(Runtime.via(Robot)) == runtime
      assert Runtime.operational_state(Robot) == :idle
    end
  end

  defp run(goal), do: run(:bad, goal)

  # The command replies to its awaiters before casting its completion to the
  # runtime, so `await/2` returning says nothing about the runtime having
  # handled it. Wait for the completion event before reading state or logs.
  defp run(command_name, goal) do
    {:ok, command} = Runtime.execute(Robot, command_name, goal)
    result = BB.Command.await(command)
    assert_receive {:bb, [:command, ^command_name, _], _}, 1_000
    result
  end
end
