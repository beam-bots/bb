# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Urdf.ImporterTest do
  use ExUnit.Case, async: true

  alias BB.Urdf.{Importer, Parser}

  @fixture_dir Path.join([__DIR__, "..", "..", "fixtures", "urdf"])

  defp import_fixture(name, module \\ MyApp.Generated) do
    {:ok, parsed} = Parser.parse_file(Path.join(@fixture_dir, name))
    {:ok, source, warnings} = Importer.to_source(parsed, module)
    {source, warnings}
  end

  describe "to_source/2" do
    test "wraps output in defmodule that uses BB" do
      {source, _warnings} = import_fixture("minimal.urdf")

      assert source =~ "defmodule MyApp.Generated do"
      assert source =~ "use BB"
      assert source =~ "settings do"
      assert source =~ "name :minimal_bot"
      assert source =~ "topology do"
      assert source =~ "link :base_link"
    end

    test "generated source compiles" do
      {source, _warnings} = import_fixture("minimal.urdf", MyApp.MinimalGen)
      assert [{MyApp.MinimalGen, _}] = Code.compile_string(source)
    end

    test "nests joints under their parent link in topology order" do
      {source, _warnings} = import_fixture("two_link_arm.urdf", MyApp.TwoLinkGen)

      # base_link contains shoulder joint, which contains upper_arm, etc.
      assert Regex.match?(
               ~r/link :base_link do.*joint :shoulder do.*link :upper_arm/s,
               source
             )

      assert Regex.match?(
               ~r/link :upper_arm do.*joint :elbow do.*link :forearm/s,
               source
             )

      assert [{MyApp.TwoLinkGen, _}] = Code.compile_string(source)
    end

    test "emits ~u sigil literals for unit-bearing values" do
      {source, _warnings} = import_fixture("two_link_arm.urdf")

      assert source =~ ~r/~u\(0\.\d+ meter\)/
      assert source =~ ~r/~u\([\d.]+ kilogram\)/
      assert source =~ ~r/~u\([\d.]+ kilogram_square_meter\)/
      assert source =~ ~r/~u\([-\d.]+ radian\)/
    end

    test "maps URDF y-axis to bb's axis rotation" do
      {source, _warnings} = import_fixture("two_link_arm.urdf")

      assert source =~ ~r/axis do\s*roll ~u\(-90 degree\)\s*end/
    end

    test "emits limit with effort/velocity only for continuous joints" do
      {source, _warnings} = import_fixture("two_link_arm.urdf")

      assert Regex.match?(
               ~r/joint :elbow do.*type :continuous.*limit do(?:(?!lower|upper).)*end/s,
               source
             )
    end

    test "omits axis, limit, and dynamics for fixed joints" do
      {source, _warnings} = import_fixture("two_link_arm.urdf")

      assert Regex.match?(
               ~r/joint :wrist do\s*type :fixed.*?link :gripper/s,
               source
             )

      refute Regex.match?(
               ~r/joint :wrist do(?:(?!end).)*axis/s,
               source
             )
    end

    test "passes through parser warnings" do
      {_source, warnings} = import_fixture("mimic_and_transmission.urdf")

      assert Enum.any?(warnings, &(&1 =~ "<safety_controller>"))
      assert Enum.any?(warnings, &(&1 =~ "<transmission>"))
    end

    test "emits a BB.Sensor.Mimic for URDF <mimic> joints" do
      {source, _warnings} = import_fixture("mimic_and_transmission.urdf", MyApp.MimicGen)

      assert source =~ "sensor :right_finger_joint_mimic"
      assert source =~ "BB.Sensor.Mimic"
      assert source =~ "source: :left_finger_joint"
      assert source =~ "multiplier: -1.0"
      assert [{MyApp.MimicGen, _}] = Code.compile_string(source)
    end

    test "omits default multiplier and offset on emitted mimic sensors" do
      xml = """
      <?xml version="1.0"?>
      <robot name="t">
        <link name="a"/>
        <link name="b"/>
        <link name="c"/>
        <joint name="j1" type="prismatic">
          <parent link="a"/>
          <child link="b"/>
          <axis xyz="1 0 0"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
        </joint>
        <joint name="j2" type="prismatic">
          <parent link="b"/>
          <child link="c"/>
          <axis xyz="1 0 0"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
          <mimic joint="j1"/>
        </joint>
      </robot>
      """

      {:ok, parsed} = Parser.parse_string(xml)
      {:ok, source, _} = Importer.to_source(parsed, MyApp.DefaultMimic)

      assert source =~ ~r/source: :j1[^,}]*\}/
      refute source =~ "multiplier:"
      refute source =~ "offset:"
    end

    test "dedupes named materials so multiple visuals can share a URDF material" do
      {source, _warnings} = import_fixture("shared_materials.urdf", MyApp.SharedGen)

      # First visual keeps the URDF material name…
      assert Regex.match?(
               ~r/link :base_link do.*?material do\s*name :grey/s,
               source
             )

      # …and the resulting module compiles (would fail if the name leaked
      # into later visuals — the DSL rejects duplicate entity names).
      assert [{MyApp.SharedGen, _}] = Code.compile_string(source)
    end

    test "emits mesh scale as a float (the default integer trips the URDF exporter)" do
      {source, _warnings} = import_fixture("shared_materials.urdf")

      assert source =~ ~r/mesh do\s*filename "package:\/\/meshes\/arm\.stl"\s*scale 1\.0/
    end

    test "drops joints whose parent link is undefined (URDF world anchor)" do
      {source, warnings} = import_fixture("world_anchor.urdf", MyApp.WorldGen)

      refute source =~ "joint :world_anchor"
      assert source =~ "joint :shoulder"
      assert Enum.any?(warnings, &(&1 =~ "dropped joint \"world_anchor\""))
      assert [{MyApp.WorldGen, _}] = Code.compile_string(source)
    end

    test "renames joints that collide with link names and rewrites mimic refs" do
      {source, _warnings} = import_fixture("name_collision.urdf", MyApp.CollideGen)

      # `gripper` is a link; the homonymous joint gets renamed.
      assert source =~ "link :gripper"
      assert source =~ "joint :gripper_joint"
      # The mimic source on `finger_joint` points to the renamed joint.
      assert source =~ "source: :gripper_joint"
      assert [{MyApp.CollideGen, _}] = Code.compile_string(source)
    end

    test "errors out on multiple root links" do
      xml = """
      <?xml version="1.0"?>
      <robot name="t">
        <link name="a"/>
        <link name="b"/>
      </robot>
      """

      {:ok, parsed} = Parser.parse_string(xml)
      assert {:error, {:multiple_root_links, _}} = Importer.to_source(parsed, X)
    end

    test "emits a transmission block for joints with a SimpleTransmission" do
      {source, _warnings} = import_fixture("simple_transmission.urdf", MyApp.TxGen)

      assert source =~ ~r/joint :shoulder_pan do.*transmission do.*reduction 101\.0/s
    end

    test "does not emit a transmission block for the default 1:1 reduction" do
      xml = """
      <?xml version="1.0"?>
      <robot name="t">
        <link name="base"/><link name="a"/>
        <joint name="j" type="revolute">
          <parent link="base"/><child link="a"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
        </joint>
        <transmission name="t1">
          <type>transmission_interface/SimpleTransmission</type>
          <joint name="j"/>
          <actuator name="m1"><mechanicalReduction>1</mechanicalReduction></actuator>
        </transmission>
      </robot>
      """

      {:ok, parsed} = Parser.parse_string(xml)
      {:ok, source, _} = Importer.to_source(parsed, MyApp.TxGenIdentity)

      refute source =~ "transmission do"
    end

    test "errors out when a joint references an undefined link" do
      xml = """
      <?xml version="1.0"?>
      <robot name="t">
        <link name="a"/>
        <joint name="bad" type="fixed">
          <parent link="a"/>
          <child link="nope"/>
        </joint>
      </robot>
      """

      {:ok, parsed} = Parser.parse_string(xml)
      assert {:error, {:undefined_link, "bad", "nope"}} = Importer.to_source(parsed, X)
    end
  end
end
