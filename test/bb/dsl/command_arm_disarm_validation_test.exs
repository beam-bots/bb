# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.CommandArmDisarmValidationTest do
  @moduledoc """
  Compile-time validation tests for the new `arm`/`disarm` command flags.

  These tests use `Code.compile_string/1` so the compile-time DSL errors can
  be captured rather than crashing the test process.
  """
  use ExUnit.Case, async: true

  alias BB.Dsl.Command, as: DslCommand
  alias BB.Dsl.Info

  defp compile_robot(name, body) do
    Code.compile_string("""
    defmodule #{name} do
      use BB
      import BB.Unit

      #{body}

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
    """)
  end

  describe "implicit flags" do
    test "BB.Command.Arm handler gets arm: true implicitly" do
      [{mod, _}] =
        compile_robot("BB.Dsl.ArmDisarmTest.ImplicitArm#{System.unique_integer([:positive])}", """
        commands do
          command :arm do
            handler BB.Command.Arm
            allowed_states [:disarmed]
          end
        end
        """)

      [arm_cmd] = mod |> Info.commands() |> Enum.filter(&is_struct(&1, DslCommand))
      assert arm_cmd.arm == true
      assert mod.__bb_arm_command__() == :arm
    end

    test "BB.Command.Disarm handler gets disarm: true implicitly" do
      [{mod, _}] =
        compile_robot(
          "BB.Dsl.ArmDisarmTest.ImplicitDisarm#{System.unique_integer([:positive])}",
          """
          commands do
            command :disarm do
              handler BB.Command.Disarm
              allowed_states [:idle]
            end
          end
          """
        )

      [disarm_cmd] = mod |> Info.commands() |> Enum.filter(&is_struct(&1, DslCommand))
      assert disarm_cmd.disarm == true
      assert mod.__bb_disarm_command__() == :disarm
    end

    test "parameterised built-in handler also gets implicit flag" do
      # `{module, opts}` handler form
      [{mod, _}] =
        compile_robot(
          "BB.Dsl.ArmDisarmTest.ParameterisedImplicit#{System.unique_integer([:positive])}",
          """
          commands do
            command :arm do
              handler {BB.Command.Arm, []}
              allowed_states [:disarmed]
            end
          end
          """
        )

      assert mod.__bb_arm_command__() == :arm
    end

    test "custom handler does not get implicit flag" do
      [{mod, _}] =
        compile_robot(
          "BB.Dsl.ArmDisarmTest.NoImplicit#{System.unique_integer([:positive])}",
          """
          commands do
            command :hello do
              handler BB.Test.ImmediateSuccessCommand
              allowed_states [:idle]
            end
          end
          """
        )

      assert mod.__bb_arm_command__() == nil
      assert mod.__bb_disarm_command__() == nil
    end
  end

  describe "duplicate flag validation" do
    test "rejects two commands with `arm true`" do
      assert_raise Spark.Error.DslError, ~r/Multiple commands have `arm true`/, fn ->
        compile_robot(
          "BB.Dsl.ArmDisarmTest.DupArm#{System.unique_integer([:positive])}",
          """
          commands do
            command :arm_one do
              handler BB.Command.Arm
              arm true
              allowed_states [:disarmed]
            end

            command :arm_two do
              handler BB.Test.ImmediateSuccessCommand
              arm true
              allowed_states [:disarmed]
            end
          end
          """
        )
      end
    end

    test "rejects two commands with `disarm true`" do
      assert_raise Spark.Error.DslError, ~r/Multiple commands have `disarm true`/, fn ->
        compile_robot(
          "BB.Dsl.ArmDisarmTest.DupDisarm#{System.unique_integer([:positive])}",
          """
          commands do
            command :disarm_one do
              handler BB.Command.Disarm
              disarm true
              allowed_states [:idle]
            end

            command :disarm_two do
              handler BB.Test.ImmediateSuccessCommand
              disarm true
              allowed_states [:idle]
            end
          end
          """
        )
      end
    end

    test "rejects a command with both `arm true` and `disarm true`" do
      assert_raise Spark.Error.DslError, ~r/both `arm true` and `disarm true`/, fn ->
        compile_robot(
          "BB.Dsl.ArmDisarmTest.BothFlags#{System.unique_integer([:positive])}",
          """
          commands do
            command :weird do
              handler BB.Test.ImmediateSuccessCommand
              arm true
              disarm true
              allowed_states [:idle, :disarmed]
            end
          end
          """
        )
      end
    end
  end

  describe "allowed_states validation" do
    test "rejects arm-flagged command without :disarmed in allowed_states" do
      assert_raise Spark.Error.DslError, ~r/must include.*:disarmed/, fn ->
        compile_robot(
          "BB.Dsl.ArmDisarmTest.ArmBadStates#{System.unique_integer([:positive])}",
          """
          commands do
            command :bad_arm do
              handler BB.Test.ImmediateSuccessCommand
              arm true
              allowed_states [:idle]
            end
          end
          """
        )
      end
    end

    test "rejects disarm-flagged command not reachable from armed states" do
      assert_raise Spark.Error.DslError, ~r/must be.*runnable from an armed state/, fn ->
        compile_robot(
          "BB.Dsl.ArmDisarmTest.DisarmBadStates#{System.unique_integer([:positive])}",
          """
          commands do
            command :bad_disarm do
              handler BB.Test.ImmediateSuccessCommand
              disarm true
              allowed_states [:disarmed]
            end
          end
          """
        )
      end
    end

    test "accepts disarm-flagged command runnable from a custom armed state" do
      [{mod, _}] =
        compile_robot(
          "BB.Dsl.ArmDisarmTest.CustomState#{System.unique_integer([:positive])}",
          """
          states do
            state :standby
          end

          commands do
            command :arm do
              handler BB.Command.Arm
              allowed_states [:disarmed]
            end

            command :soft_disarm do
              handler BB.Test.ImmediateSuccessCommand
              disarm true
              allowed_states [:standby]
            end
          end
          """
        )

      assert mod.__bb_disarm_command__() == :soft_disarm
    end
  end
end
