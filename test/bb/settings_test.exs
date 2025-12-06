# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.SettingsTest do
  use ExUnit.Case, async: true

  alias BB.Dsl.Info

  describe "registry_options" do
    defmodule DefaultOptionsRobot do
      @moduledoc false
      use BB

      topology do
        link :base do
        end
      end
    end

    defmodule CustomOptionsRobot do
      @moduledoc false
      use BB

      settings do
        registry_options partitions: 4
      end

      topology do
        link :base do
        end
      end
    end

    defmodule EmptyOptionsRobot do
      @moduledoc false
      use BB

      settings do
        registry_options []
      end

      topology do
        link :base do
        end
      end
    end

    test "defaults to partitions based on scheduler count" do
      settings = Info.settings(DefaultOptionsRobot)

      assert settings.registry_options == [partitions: System.schedulers_online()]
    end

    test "can be overridden with custom options" do
      settings = Info.settings(CustomOptionsRobot)

      assert settings.registry_options == [partitions: 4]
    end

    test "can be set to empty list to disable partitioning" do
      settings = Info.settings(EmptyOptionsRobot)

      assert settings.registry_options == []
    end

    test "robot starts successfully with custom registry options" do
      pid = start_supervised!(CustomOptionsRobot)
      assert Process.alive?(pid)
    end

    test "robot starts successfully with empty registry options" do
      pid = start_supervised!(EmptyOptionsRobot)
      assert Process.alive?(pid)
    end
  end
end
