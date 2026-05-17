# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

# credo:disable-for-next-line Credo.Check.Readability.ModuleNames
defmodule BB.Igniter.Upgrade.V0_16Test do
  use ExUnit.Case, async: true

  import Igniter.Test

  alias BB.Igniter.Upgrade.V0_16

  describe "remove_auto_disarm_on_error/2" do
    test "removes the setting from a robot module" do
      [
        files: %{
          "lib/my_app/robot.ex" => """
          defmodule MyApp.Robot do
            use BB

            settings do
              auto_disarm_on_error false
            end

            topology do
              link :base
            end
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.remove_auto_disarm_on_error([])
      |> assert_has_patch("lib/my_app/robot.ex", """
      |    settings do
      -|      auto_disarm_on_error false
      |    end
      """)
    end

    test "leaves modules that don't use BB untouched" do
      [
        files: %{
          "lib/some_other.ex" => """
          defmodule SomeOther do
            def auto_disarm_on_error(_), do: :ok
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.remove_auto_disarm_on_error([])
      |> assert_unchanged("lib/some_other.ex")
    end
  end

  describe "rename_bb_cldr_unit_alias/2" do
    test "rewrites a bare alias" do
      [
        files: %{
          "lib/my_app/foo.ex" => """
          defmodule MyApp.Foo do
            alias BB.Cldr.Unit

            def compat?(u), do: Unit.compatible?(u, "meter")
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.rename_bb_cldr_unit_alias([])
      |> assert_has_patch("lib/my_app/foo.ex", """
      -|  alias BB.Cldr.Unit
      +|  alias BB.Unit
      """)
    end

    test "preserves the `as:` clause" do
      [
        files: %{
          "lib/my_app/bar.ex" => """
          defmodule MyApp.Bar do
            alias BB.Cldr.Unit, as: CldrUnit

            def name(u), do: u.name
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.rename_bb_cldr_unit_alias([])
      |> assert_has_patch("lib/my_app/bar.ex", """
      -|  alias BB.Cldr.Unit, as: CldrUnit
      +|  alias BB.Unit, as: CldrUnit
      """)
    end
  end

  describe "rewrite_cldr_unit_calls/2" do
    test "rewrites `Cldr.Unit.new!` in a BB.Actuator module, reversing args and dashing the atom" do
      [
        files: %{
          "lib/my_app/servo.ex" => """
          defmodule MyApp.Servo do
            use BB.Actuator

            def disarm(_), do: :ok

            def init(_) do
              {:ok, %{torque: Cldr.Unit.new!(:newton_meter, 5)}}
            end
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.rewrite_cldr_unit_calls([])
      |> assert_has_patch("lib/my_app/servo.ex", """
      -|      {:ok, %{torque: Cldr.Unit.new!(:newton_meter, 5)}}
      +|      {:ok, %{torque: Localize.Unit.new!(5, "newton-meter")}}
      """)
    end

    test "rewrites `Cldr.Unit.convert!` in a BB.Sensor module" do
      [
        files: %{
          "lib/my_app/sensor.ex" => """
          defmodule MyApp.Sensor do
            use BB.Sensor

            def init(_) do
              v = some_unit() |> Cldr.Unit.convert!(:radian_per_second)
              {:ok, v}
            end

            defp some_unit, do: nil
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.rewrite_cldr_unit_calls([])
      |> assert_has_patch("lib/my_app/sensor.ex", """
      -|      v = some_unit() |> Cldr.Unit.convert!(:radian_per_second)
      +|      v = some_unit() |> Localize.Unit.convert!("radian-per-second")
      """)
    end

    test "leaves non-BB modules untouched" do
      [
        files: %{
          "lib/random.ex" => """
          defmodule Random do
            def thing, do: Cldr.Unit.new!(:meter, 5)
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.rewrite_cldr_unit_calls([])
      |> assert_unchanged("lib/random.ex")
    end
  end

  describe "rewrite_cldr_unit_struct_patterns/2" do
    test "rewrites `%Cldr.Unit{}` to `%Localize.Unit{}` with name field rename" do
      [
        files: %{
          "lib/my_app/sensor.ex" => """
          defmodule MyApp.Sensor do
            use BB.Sensor

            def init(_) do
              {:ok, %Cldr.Unit{unit: :meter, value: 1}}
            end
          end
          """
        }
      ]
      |> test_project()
      |> V0_16.rewrite_cldr_unit_struct_patterns([])
      |> assert_has_patch("lib/my_app/sensor.ex", """
      -|      {:ok, %Cldr.Unit{unit: :meter, value: 1}}
      +|      {:ok, %Localize.Unit{name: "meter", value: 1}}
      """)
    end
  end

  describe "add_release_notice/2" do
    test "adds the migration notice" do
      igniter =
        []
        |> test_project()
        |> V0_16.add_release_notice([])

      assert Enum.any?(igniter.notices, fn n ->
               String.contains?(n, "documentation/how-to/upgrade-to-0.16.md")
             end)
    end
  end
end
