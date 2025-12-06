# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.DslTest do
  use ExUnit.Case, async: true
  alias BB.Dsl.{Info, Link}

  describe "robot naming" do
    defmodule MostBasicRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
        end
      end
    end

    test "defaults the robot name from module name" do
      assert {:ok, :most_basic_robot} = Info.settings_name(MostBasicRobot)
    end

    defmodule ExplicitlyNamedRobot do
      @moduledoc false
      use BB

      settings do
        name :my_custom_robot
      end

      topology do
        link :base_link do
        end
      end
    end

    test "explicit robot name is used when provided" do
      assert {:ok, :my_custom_robot} = Info.settings_name(ExplicitlyNamedRobot)
    end
  end

  describe "root link" do
    defmodule BasicLinkRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
        end
      end
    end

    test "robot contains a single root link" do
      assert [link] = Info.topology(BasicLinkRobot)
      assert is_struct(link, Link)
      assert link.name == :base_link
    end

    defmodule AutoNamedLinkRobot do
      @moduledoc false
      use BB

      topology do
        link do
        end
      end
    end

    test "link name auto-generates when not provided" do
      assert [link] = Info.topology(AutoNamedLinkRobot)
      assert link.name == :link_0
    end
  end
end
