# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Urdf.ParserTest do
  use ExUnit.Case, async: true

  alias BB.Urdf.Parser

  @fixture_dir Path.join([__DIR__, "..", "..", "fixtures", "urdf"])

  describe "parse_file/1" do
    test "parses a minimal robot with no joints" do
      {:ok, robot} = Parser.parse_file(Path.join(@fixture_dir, "minimal.urdf"))

      assert robot.name == "minimal_bot"
      assert [%{name: "base_link"}] = robot.links
      assert robot.joints == []
      assert robot.warnings == []
    end

    test "parses links, joints, visuals, materials and inertials" do
      {:ok, robot} = Parser.parse_file(Path.join(@fixture_dir, "two_link_arm.urdf"))

      assert robot.name == "two_link_arm"
      assert length(robot.links) == 4
      assert length(robot.joints) == 3

      base = Enum.find(robot.links, &(&1.name == "base_link"))
      assert base.visual.geometry == {:box, %{size: {0.2, 0.2, 0.05}}}
      assert base.visual.origin == %{xyz: {0.0, 0.0, 0.1}, rpy: {0.0, 0.0, 0.0}}
      assert base.visual.material.color == %{red: 1.0, green: 0.0, blue: 0.0, alpha: 1.0}
      assert [%{name: "base_col", geometry: {:sphere, %{radius: 0.1}}}] = base.collisions
      assert base.inertial.mass == 1.5

      shoulder = Enum.find(robot.joints, &(&1.name == "shoulder"))
      assert shoulder.type == :revolute
      assert shoulder.parent == "base_link"
      assert shoulder.child == "upper_arm"
      assert shoulder.axis == {0.0, 1.0, 0.0}
      assert shoulder.limit.lower == -1.57
      assert shoulder.dynamics == %{damping: 0.5, friction: 0.1}
    end

    test "collects warnings for unsupported features" do
      {:ok, robot} = Parser.parse_file(Path.join(@fixture_dir, "mimic_and_transmission.urdf"))

      assert Enum.any?(robot.warnings, &(&1 =~ "<mimic>"))
      assert Enum.any?(robot.warnings, &(&1 =~ "<safety_controller>"))
      assert Enum.any?(robot.warnings, &(&1 =~ "<transmission>"))
      assert Enum.any?(robot.warnings, &(&1 =~ "<gazebo>"))
    end

    test "resolves top-level material references inside visuals" do
      {:ok, robot} = Parser.parse_file(Path.join(@fixture_dir, "two_link_arm.urdf"))

      base = Enum.find(robot.links, &(&1.name == "base_link"))
      assert base.visual.material.name == "red"
      assert base.visual.material.color == %{red: 1.0, green: 0.0, blue: 0.0, alpha: 1.0}
    end

    test "returns an error for missing files" do
      assert {:error, :enoent} = Parser.parse_file("/no/such/file.urdf")
    end
  end

  describe "parse_string/1" do
    test "returns an error for malformed XML" do
      assert {:error, {:xml_parse_error, _}} = Parser.parse_string("<robot><link></robot>")
    end

    test "parses cylinder geometry with radius and length" do
      xml = """
      <?xml version="1.0"?>
      <robot name="t"><link name="a">
        <collision><geometry><cylinder radius="0.1" length="0.5"/></geometry></collision>
      </link></robot>
      """

      {:ok, robot} = Parser.parse_string(xml)
      [link] = robot.links
      [collision] = link.collisions
      assert collision.geometry == {:cylinder, %{radius: 0.1, length: 0.5}}
    end

    test "parses mesh geometry with filename and scale" do
      xml = """
      <?xml version="1.0"?>
      <robot name="t"><link name="a">
        <visual><geometry><mesh filename="package://thing/mesh.stl" scale="0.5 0.5 0.5"/></geometry></visual>
      </link></robot>
      """

      {:ok, robot} = Parser.parse_string(xml)
      [link] = robot.links
      assert link.visual.geometry == {:mesh, %{filename: "package://thing/mesh.stl", scale: 0.5}}
    end
  end
end
