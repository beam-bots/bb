# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ReservedStateNamesTest do
  @moduledoc """
  The safety system's states can't be redeclared as operational ones.

  Compiles via `Code.compile_string/1` so the DSL error can be captured
  rather than taking the test process down with it.
  """
  use ExUnit.Case, async: true

  alias BB.Dsl.Info
  alias BB.Dsl.State

  defp compile_robot(name, states_block) do
    Code.compile_string("""
    defmodule #{name} do
      use BB

      #{states_block}

      topology do
        link :base
      end
    end
    """)
  end

  defp unique(prefix), do: "#{prefix}#{System.unique_integer([:positive])}"

  test "every reserved name is rejected" do
    for reserved <- State.reserved_names() do
      assert_raise Spark.Error.DslError,
                   ~r/Cannot define a state named #{inspect(reserved)}/,
                   fn ->
                     compile_robot(unique("BB.Dsl.ReservedStateNamesTest.Reserved"), """
                     states do
                       state #{inspect(reserved)}
                     end
                     """)
                   end
    end
  end

  test "the error says why, not just that" do
    assert_raise Spark.Error.DslError, ~r/belongs to the safety system/, fn ->
      compile_robot(unique("BB.Dsl.ReservedStateNamesTest.Why"), """
      states do
        state :disarming
      end
      """)
    end
  end

  test "a reserved name is rejected even alongside valid states" do
    assert_raise Spark.Error.DslError, ~r/Cannot define a state named :error/, fn ->
      compile_robot(unique("BB.Dsl.ReservedStateNamesTest.Mixed"), """
      states do
        state :recording
        state :error
      end
      """)
    end
  end

  test ":disarmed is a built-in, not a reserved name, so it can be redeclared" do
    [{robot, _} | _] =
      compile_robot(unique("BB.Dsl.ReservedStateNamesTest.Disarmed"), """
      states do
        state :disarmed do
          doc "Parked in the charger"
        end
      end
      """)

    disarmed = Enum.find(Info.states(robot), &(&1.name == :disarmed))

    assert disarmed.doc == "Parked in the charger"
    assert Enum.count(Info.state_names(robot), &(&1 == :disarmed)) == 1
  end
end
