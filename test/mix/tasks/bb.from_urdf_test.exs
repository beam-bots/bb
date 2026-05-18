# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Bb.FromUrdfTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  @fixture_dir Path.join([__DIR__, "..", "..", "fixtures", "urdf"])

  test "generates a robot module from a minimal URDF" do
    urdf = Path.join(@fixture_dir, "minimal.urdf")

    test_project()
    |> Igniter.compose_task("bb.from_urdf", [urdf, "--module", "Test.Robot"])
    |> assert_creates("lib/test/robot.ex")
  end

  test "generated module uses BB and emits the URDF robot name" do
    urdf = Path.join(@fixture_dir, "minimal.urdf")

    igniter =
      test_project()
      |> Igniter.compose_task("bb.from_urdf", [urdf, "--module", "Test.Robot"])

    {_, source} = Rewrite.source(igniter.rewrite, "lib/test/robot.ex")
    assert source.content =~ "use BB"
    assert source.content =~ ~r/name[ (]:minimal_bot/
    assert source.content =~ ~r/link[ (]:base_link/
  end

  test "produces a warning for unsupported URDF features" do
    urdf = Path.join(@fixture_dir, "mimic_and_transmission.urdf")

    igniter =
      test_project()
      |> Igniter.compose_task("bb.from_urdf", [urdf, "--module", "Test.Robot"])

    assert Enum.any?(igniter.warnings, &(&1 =~ "<mimic>"))
    assert Enum.any?(igniter.warnings, &(&1 =~ "<transmission>"))
  end

  test "records an issue when the URDF file is missing" do
    igniter =
      test_project()
      |> Igniter.compose_task("bb.from_urdf", ["/no/such/file.urdf", "--module", "Test.Robot"])

    assert Enum.any?(igniter.issues, &(&1 =~ "Could not parse URDF"))
  end
end
