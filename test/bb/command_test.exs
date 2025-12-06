# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.CommandTest do
  use ExUnit.Case, async: true
  alias BB.Dsl.{Command, Info}

  defmodule TestHandler do
    @moduledoc false
  end

  describe "command DSL" do
    defmodule SingleCommandRobot do
      @moduledoc false
      use BB

      commands do
        command :arm do
          handler(BB.CommandTest.TestHandler)
        end
      end

      topology do
        link :base_link do
        end
      end
    end

    test "command defined at robot level" do
      commands = Info.commands(SingleCommandRobot)
      assert length(commands) == 1

      [command] = commands
      assert is_struct(command, Command)
      assert command.name == :arm
      assert command.handler == BB.CommandTest.TestHandler
      assert command.timeout == :infinity
      assert command.allowed_states == [:idle]
    end
  end

  describe "command with options" do
    defmodule CommandWithOptionsRobot do
      @moduledoc false
      use BB

      commands do
        command :navigate do
          handler(BB.CommandTest.TestHandler)
          timeout(30_000)
          allowed_states([:idle, :executing])
        end
      end

      topology do
        link :base_link do
        end
      end
    end

    test "command with custom timeout and allowed_states" do
      [command] = Info.commands(CommandWithOptionsRobot)
      assert command.name == :navigate
      assert command.timeout == 30_000
      assert command.allowed_states == [:idle, :executing]
    end
  end

  describe "command with arguments" do
    defmodule CommandWithArgumentsRobot do
      @moduledoc false
      use BB

      commands do
        command :navigate_to_pose do
          handler(BB.CommandTest.TestHandler)

          argument :target_pose, :map do
            required(true)
            doc("The target pose to navigate to")
          end

          argument :tolerance, :float do
            default(0.1)
            doc("Position tolerance in meters")
          end
        end
      end

      topology do
        link :base_link do
        end
      end
    end

    test "command with arguments" do
      [command] = Info.commands(CommandWithArgumentsRobot)
      assert command.name == :navigate_to_pose
      assert length(command.arguments) == 2

      target_pose = Enum.find(command.arguments, &(&1.name == :target_pose))
      assert target_pose.type == :map
      assert target_pose.required == true
      assert target_pose.doc == "The target pose to navigate to"

      tolerance = Enum.find(command.arguments, &(&1.name == :tolerance))
      assert tolerance.type == :float
      assert tolerance.required == false
      assert tolerance.default == 0.1
      assert tolerance.doc == "Position tolerance in meters"
    end
  end

  describe "multiple commands" do
    defmodule MultipleCommandsRobot do
      @moduledoc false
      use BB

      commands do
        command :arm do
          handler(BB.CommandTest.TestHandler)
          allowed_states([:disarmed, :idle])
        end

        command :disarm do
          handler(BB.CommandTest.TestHandler)
          allowed_states([:idle, :executing])
        end

        command :navigate do
          handler(BB.CommandTest.TestHandler)
          allowed_states([:idle])
        end
      end

      topology do
        link :base_link do
        end
      end
    end

    test "multiple commands defined" do
      commands = Info.commands(MultipleCommandsRobot)
      assert length(commands) == 3

      names = Enum.map(commands, & &1.name)
      assert :arm in names
      assert :disarm in names
      assert :navigate in names
    end
  end

  describe "command name uniqueness" do
    test "allows command with same name as sensor (not in registry)" do
      defmodule CommandSensorSameName do
        @moduledoc false
        use BB

        sensors do
          sensor :navigate, SomeSensor
        end

        commands do
          command :navigate do
            handler(BB.CommandTest.TestHandler)
          end
        end

        topology do
          link :base_link do
          end
        end
      end

      # Both should exist without error - commands aren't in the registry
      assert length(Info.sensors(CommandSensorSameName)) == 1
      assert length(Info.commands(CommandSensorSameName)) == 1
    end
  end
end
