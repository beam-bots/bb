# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Bb.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  test "creates a robot module with default name" do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> assert_creates("lib/test/robot.ex", """
    defmodule Test.Robot do
      use BB

      commands do
        command :arm do
          handler(BB.Command.Arm)
          allowed_states([:disarmed])
        end

        command :disarm do
          handler(BB.Command.Disarm)
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

  test "adds the robot to the supervision tree" do
    igniter =
      test_project()
      |> Igniter.compose_task("bb.install")

    assert_creates(igniter, "lib/test/application.ex")

    {_, source} = Rewrite.source(igniter.rewrite, "lib/test/application.ex")
    assert source.content =~ "{Test.Robot, []}"
  end

  test "adds bb to formatter imports" do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> assert_has_patch(".formatter.exs", """
    + |  import_deps: [:bb]
    """)
  end

  test "installation is idempotent" do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
    |> Igniter.compose_task("bb.install")
    |> assert_unchanged()
  end
end
