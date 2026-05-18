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

      assert Enum.any?(warnings, &(&1 =~ "<mimic>"))
      assert Enum.any?(warnings, &(&1 =~ "<safety_controller>"))
      assert Enum.any?(warnings, &(&1 =~ "<transmission>"))
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
