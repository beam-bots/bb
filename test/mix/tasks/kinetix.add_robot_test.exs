# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Kinetix.AddRobotTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  test "creates a robot module with default name" do
    test_project()
    |> Igniter.compose_task("kinetix.add_robot")
    |> assert_creates("lib/test/robot.ex")
  end

  test "creates a robot module with custom name" do
    test_project()
    |> Igniter.compose_task("kinetix.add_robot", ["--robot", "Test.Robots.MainRobot"])
    |> assert_creates("lib/test/robots/main_robot.ex", """
    defmodule Test.Robots.MainRobot do
      use Kinetix

      commands do
        command :arm do
          handler(Kinetix.Command.Arm)
          allowed_states([:disarmed])
        end

        command :disarm do
          handler(Kinetix.Command.Disarm)
          allowed_states([:idle])
        end
      end

      topology do
        link :base_link do
        end
      end
    end
    """)
  end

  test "adds robot to supervision tree" do
    igniter =
      test_project()
      |> Igniter.compose_task("kinetix.add_robot", ["--robot", "Test.MyRobot"])

    assert_creates(igniter, "lib/test/application.ex")

    {_, source} = Rewrite.source(igniter.rewrite, "lib/test/application.ex")
    assert source.content =~ "{Test.MyRobot, []}"
  end

  test "can add multiple robots" do
    test_project()
    |> Igniter.compose_task("kinetix.add_robot", ["--robot", "Test.Robot1"])
    |> apply_igniter!()
    |> Igniter.compose_task("kinetix.add_robot", ["--robot", "Test.Robot2"])
    |> assert_creates("lib/test/robot2.ex")
  end
end
