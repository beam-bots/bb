# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IgniterTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  defp project_with_robot do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
  end

  describe "robot_module/1" do
    test "defaults to {AppPrefix}.Robot when --robot is not given" do
      igniter = put_options(test_project(), [])
      assert BB.Igniter.robot_module(igniter) == Test.Robot
    end

    test "parses --robot when provided" do
      igniter = put_options(test_project(), robot: "MyApp.Arms.Left")
      assert BB.Igniter.robot_module(igniter) == MyApp.Arms.Left
    end
  end

  describe "add_controller/4" do
    test "adds a controller entry to the controllers section" do
      controller = "controller :my_controller, {SomeApp.Controller, port: \"/dev/ttyUSB0\"}\n"

      project_with_robot()
      |> BB.Igniter.add_controller(Test.Robot, :my_controller, controller)
      |> assert_has_patch("lib/test/robot.ex", """
      + |  controllers do
      + |    controller(:my_controller, {SomeApp.Controller, port: "/dev/ttyUSB0"})
      + |  end
      """)
    end

    test "is idempotent on controller name" do
      controller = "controller :my_controller, {SomeApp.Controller, port: \"/dev/ttyUSB0\"}\n"

      project_with_robot()
      |> BB.Igniter.add_controller(Test.Robot, :my_controller, controller)
      |> apply_igniter!()
      |> BB.Igniter.add_controller(Test.Robot, :my_controller, controller)
      |> assert_unchanged()
    end
  end

  describe "add_parameter_bridge/4" do
    test "adds a bridge entry to the parameters section" do
      bridge = "bridge :my_bridge, {SomeApp.Bridge, controller: :my_controller}\n"

      project_with_robot()
      |> BB.Igniter.add_parameter_bridge(Test.Robot, :my_bridge, bridge)
      |> assert_has_patch("lib/test/robot.ex", """
      + |  parameters do
      + |    bridge(:my_bridge, {SomeApp.Bridge, controller: :my_controller})
      + |  end
      """)
    end

    test "is idempotent on bridge name" do
      bridge = "bridge :my_bridge, {SomeApp.Bridge, controller: :my_controller}\n"

      project_with_robot()
      |> BB.Igniter.add_parameter_bridge(Test.Robot, :my_bridge, bridge)
      |> apply_igniter!()
      |> BB.Igniter.add_parameter_bridge(Test.Robot, :my_bridge, bridge)
      |> assert_unchanged()
    end
  end

  describe "add_param_group/4" do
    test "wraps the body in the requested nested groups" do
      project_with_robot()
      |> BB.Igniter.add_param_group(
        Test.Robot,
        [:config, :widget],
        "param :speed, type: :integer, default: 100\n"
      )
      |> assert_has_patch("lib/test/robot.ex", """
      + |  parameters do
      + |    group :config do
      + |      group :widget do
      + |        param(:speed, type: :integer, default: 100)
      + |      end
      + |    end
      + |  end
      """)
    end

    test "is idempotent on the full group path" do
      body = "param :speed, type: :integer, default: 100\n"

      project_with_robot()
      |> BB.Igniter.add_param_group(Test.Robot, [:config, :widget], body)
      |> apply_igniter!()
      |> BB.Igniter.add_param_group(Test.Robot, [:config, :widget], body)
      |> assert_unchanged()
    end
  end

  describe "set_robot_opts/3" do
    test "sets opts on the robot's child spec in the application module" do
      project_with_robot()
      |> BB.Igniter.set_robot_opts(Test.Robot, params: [config: [widget: [speed: 100]]])
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [widget: [speed: 100]]]]}]
      ''')
    end

    test "replaces existing opts on subsequent calls" do
      project_with_robot()
      |> BB.Igniter.set_robot_opts(Test.Robot, params: [config: [widget: [speed: 100]]])
      |> apply_igniter!()
      |> BB.Igniter.set_robot_opts(Test.Robot, params: [config: [widget: [speed: 100]]])
      |> assert_unchanged()
    end
  end

  defp put_options(igniter, options) do
    %{igniter | args: %{igniter.args | options: options}}
  end
end
