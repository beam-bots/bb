# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Kinetix.ToUrdfTest do
  use ExUnit.Case, async: true
  import ExUnit.CaptureIO

  alias Mix.Tasks.Kinetix.ToUrdf

  @module_name "Mix.Tasks.Kinetix.ToUrdfTest.TestRobot"

  defmodule TestRobot do
    use Kinetix

    topology do
      link :base do
        joint :joint1 do
          type(:fixed)

          link(:link1)
        end
      end
    end
  end

  describe "run/1" do
    test "outputs URDF to stdout when no --output specified" do
      output =
        capture_io(fn ->
          ToUrdf.run([@module_name])
        end)

      assert output =~ ~s(<robot name="#{@module_name}">)
      assert output =~ ~s(<link name="base"/>)
      assert output =~ ~s(<link name="link1"/>)
      assert output =~ ~s(<joint name="joint1" type="fixed">)
    end

    test "outputs URDF to stdout with -o -" do
      output =
        capture_io(fn ->
          ToUrdf.run([@module_name, "-o", "-"])
        end)

      assert output =~ ~s(<robot name="#{@module_name}">)
    end

    test "writes URDF to file with --output" do
      path = Path.join(System.tmp_dir!(), "test_robot_#{:rand.uniform(100_000)}.urdf")

      on_exit(fn -> File.rm(path) end)

      output =
        capture_io(fn ->
          ToUrdf.run([@module_name, "--output", path])
        end)

      assert output =~ "Wrote URDF to #{path}"
      assert File.exists?(path)

      content = File.read!(path)
      assert content =~ ~s(<robot name="#{@module_name}">)
    end

    test "accepts short -o flag for output" do
      path = Path.join(System.tmp_dir!(), "test_robot_short_#{:rand.uniform(100_000)}.urdf")

      on_exit(fn -> File.rm(path) end)

      capture_io(fn ->
        ToUrdf.run([@module_name, "-o", path])
      end)

      assert File.exists?(path)
    end

    test "shows error for missing module argument" do
      assert catch_exit(
               capture_io(:stderr, fn ->
                 ToUrdf.run([])
               end)
             ) == {:shutdown, 1}
    end

    test "shows error for non-existent module" do
      assert catch_exit(
               capture_io(:stderr, fn ->
                 ToUrdf.run(["NonExistent.Module"])
               end)
             ) == {:shutdown, 1}
    end

    test "shows error for module without robot/0" do
      assert catch_exit(
               capture_io(:stderr, fn ->
                 ToUrdf.run(["Enum"])
               end)
             ) == {:shutdown, 1}
    end

    test "shows error for unknown options" do
      assert catch_exit(
               capture_io(:stderr, fn ->
                 ToUrdf.run([@module_name, "--unknown"])
               end)
             ) == {:shutdown, 1}
    end

    test "shows error for too many arguments" do
      assert catch_exit(
               capture_io(:stderr, fn ->
                 ToUrdf.run(["Module1", "Module2"])
               end)
             ) == {:shutdown, 1}
    end
  end
end
