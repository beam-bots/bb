# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.ControllerTest do
  use ExUnit.Case, async: true
  alias Kinetix.Dsl.{Controller, Info}
  alias Kinetix.Process, as: KinetixProcess

  defmodule TestGenServer do
    use GenServer

    def init(opts) do
      {:ok, opts}
    end

    def handle_call(:get_state, _from, state) do
      {:reply, state, state}
    end
  end

  describe "controller DSL" do
    defmodule SingleControllerRobot do
      @moduledoc false
      use Kinetix

      controllers do
        controller(:path_follower, Kinetix.ControllerTest.TestGenServer)
      end

      topology do
        link :base_link do
        end
      end
    end

    test "controller defined at robot level" do
      controllers = Info.controllers(SingleControllerRobot)
      assert length(controllers) == 1

      [controller] = controllers
      assert is_struct(controller, Controller)
      assert controller.name == :path_follower
      assert controller.child_spec == Kinetix.ControllerTest.TestGenServer
    end
  end

  describe "controller with options" do
    defmodule ControllerWithOptionsRobot do
      @moduledoc false
      use Kinetix

      controllers do
        controller(:velocity_smoother, {Kinetix.ControllerTest.TestGenServer, max_accel: 1.0})
      end

      topology do
        link :base_link do
        end
      end
    end

    test "controller with module and args" do
      [controller] = Info.controllers(ControllerWithOptionsRobot)
      assert controller.name == :velocity_smoother
      assert controller.child_spec == {Kinetix.ControllerTest.TestGenServer, [max_accel: 1.0]}
    end
  end

  describe "multiple controllers" do
    defmodule MultipleControllersRobot do
      @moduledoc false
      use Kinetix

      controllers do
        controller(:path_follower, Kinetix.ControllerTest.TestGenServer)
        controller(:velocity_smoother, {Kinetix.ControllerTest.TestGenServer, max_accel: 1.0})
      end

      topology do
        link :base_link do
        end
      end
    end

    test "multiple controllers defined" do
      controllers = Info.controllers(MultipleControllersRobot)
      assert length(controllers) == 2

      names = Enum.map(controllers, & &1.name)
      assert :path_follower in names
      assert :velocity_smoother in names
    end
  end

  describe "controllers in supervision tree" do
    defmodule SupervisedControllerRobot do
      @moduledoc false
      use Kinetix

      controllers do
        controller(:test_controller, {Kinetix.ControllerTest.TestGenServer, value: 42})
      end

      topology do
        link :base_link do
        end
      end
    end

    test "controller is started and registered" do
      start_supervised!(SupervisedControllerRobot)

      controller_pid = KinetixProcess.whereis(SupervisedControllerRobot, :test_controller)
      assert is_pid(controller_pid)
      assert Process.alive?(controller_pid)
    end

    test "controller receives kinetix context in init" do
      start_supervised!(SupervisedControllerRobot)

      controller_pid = KinetixProcess.whereis(SupervisedControllerRobot, :test_controller)
      state = GenServer.call(controller_pid, :get_state)

      assert state[:value] == 42
      assert state[:kinetix] == %{robot: SupervisedControllerRobot, path: [:test_controller]}
    end
  end

  describe "name uniqueness" do
    test "rejects controller with same name as sensor" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:duplicate/, fn ->
        defmodule ControllerSensorSameName do
          use Kinetix

          sensors do
            sensor :duplicate, Kinetix.ControllerTest.TestGenServer
          end

          controllers do
            controller(:duplicate, Kinetix.ControllerTest.TestGenServer)
          end

          topology do
            link :base_link do
            end
          end
        end
      end
    end

    test "rejects controller with same name as link" do
      assert_raise Spark.Error.DslError, ~r/names are used more than once.*:base_link/, fn ->
        defmodule ControllerLinkSameName do
          use Kinetix

          controllers do
            controller(:base_link, Kinetix.ControllerTest.TestGenServer)
          end

          topology do
            link :base_link do
            end
          end
        end
      end
    end
  end
end
