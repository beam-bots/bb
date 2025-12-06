# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.ParameterTest do
  use ExUnit.Case, async: true

  alias BB.Parameter
  alias BB.Parameter.Changed, as: ParameterChanged

  defmodule TestController do
    @moduledoc false
    @behaviour BB.Parameter

    @impl BB.Parameter
    def param_schema do
      Spark.Options.new!(
        kp: [type: :float, required: true, doc: "Proportional gain"],
        ki: [type: :float, default: 0.0, doc: "Integral gain"],
        kd: [type: :float, default: 0.0, doc: "Derivative gain"]
      )
    end
  end

  defmodule TestRobot do
    @moduledoc false
    use BB

    topology do
      link :base_link do
      end
    end
  end

  describe "implements?/1" do
    test "returns true for modules implementing BB.Parameter" do
      assert Parameter.implements?(TestController)
    end

    test "returns false for modules not implementing BB.Parameter" do
      refute Parameter.implements?(String)
    end
  end

  describe "register/3" do
    test "registers component parameters with defaults" do
      start_supervised!(TestRobot)

      assert :ok = Parameter.register(TestRobot, [:controller, :pid], TestController)

      # Defaults should be set
      assert {:ok, ki} = Parameter.get(TestRobot, [:controller, :pid, :ki])
      assert ki == 0.0
      assert {:ok, kd} = Parameter.get(TestRobot, [:controller, :pid, :kd])
      assert kd == 0.0
    end

    test "returns error for non-parameter modules" do
      start_supervised!(TestRobot)

      assert {:error, {:not_a_parameter_component, String}} =
               Parameter.register(TestRobot, [:test], String)
    end
  end

  describe "get/2" do
    test "returns error for unregistered parameters" do
      start_supervised!(TestRobot)

      assert {:error, :not_found} = Parameter.get(TestRobot, [:nonexistent])
    end

    test "returns value for registered parameters" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      assert {:ok, value} = Parameter.get(TestRobot, [:controller, :pid, :ki])
      assert value == 0.0
    end
  end

  describe "get!/2" do
    test "raises for unregistered parameters" do
      start_supervised!(TestRobot)

      assert_raise ArgumentError, ~r/parameter not found/, fn ->
        Parameter.get!(TestRobot, [:nonexistent])
      end
    end

    test "returns value for registered parameters" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      value = Parameter.get!(TestRobot, [:controller, :pid, :ki])
      assert value == 0.0
    end
  end

  describe "set/3" do
    test "rejects setting unregistered parameters" do
      start_supervised!(TestRobot)

      assert {:error, {:unregistered_parameter, [:nonexistent, :param]}} =
               Parameter.set(TestRobot, [:nonexistent, :param], 1.0)
    end

    test "rejects parameters not in schema" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      assert {:error, {:unknown_parameter, :nonexistent}} =
               Parameter.set(TestRobot, [:controller, :pid, :nonexistent], 1.0)
    end

    test "validates parameter type" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      # kp expects :float, not string
      assert {:error, _} = Parameter.set(TestRobot, [:controller, :pid, :kp], "not a float")
    end

    test "accepts valid parameter values" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      assert :ok = Parameter.set(TestRobot, [:controller, :pid, :kp], 2.5)
      assert {:ok, 2.5} = Parameter.get(TestRobot, [:controller, :pid, :kp])
    end
  end

  describe "set_many/2" do
    test "sets multiple parameters atomically" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      assert :ok =
               Parameter.set_many(TestRobot, [
                 {[:controller, :pid, :kp], 1.0},
                 {[:controller, :pid, :ki], 0.1},
                 {[:controller, :pid, :kd], 0.01}
               ])

      assert {:ok, 1.0} = Parameter.get(TestRobot, [:controller, :pid, :kp])
      assert {:ok, 0.1} = Parameter.get(TestRobot, [:controller, :pid, :ki])
      assert {:ok, 0.01} = Parameter.get(TestRobot, [:controller, :pid, :kd])
    end

    test "rejects batch if any parameter invalid" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      # Set initial values
      Parameter.set(TestRobot, [:controller, :pid, :kp], 1.0)

      # Try to set batch with one invalid value
      assert {:error, _errors} =
               Parameter.set_many(TestRobot, [
                 {[:controller, :pid, :kp], 2.0},
                 {[:controller, :pid, :ki], "invalid"}
               ])

      # Original value should be unchanged (atomic rollback)
      assert {:ok, 1.0} = Parameter.get(TestRobot, [:controller, :pid, :kp])
    end
  end

  describe "list/2" do
    test "returns empty list when no parameters registered" do
      start_supervised!(TestRobot)

      assert [] = Parameter.list(TestRobot)
    end

    test "returns all registered parameters" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      params = Parameter.list(TestRobot)

      # Should have ki and kd (defaults were set)
      paths = Enum.map(params, fn {path, _meta} -> path end)
      assert [:controller, :pid, :ki] in paths
      assert [:controller, :pid, :kd] in paths
    end

    test "filters by prefix" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)
      Parameter.register(TestRobot, [:sensor, :imu], TestController)

      params = Parameter.list(TestRobot, prefix: [:controller])

      paths = Enum.map(params, fn {path, _meta} -> path end)

      # Should only have controller params
      assert Enum.all?(paths, fn path -> hd(path) == :controller end)
    end

    test "includes metadata from schema" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      params = Parameter.list(TestRobot)

      {_path, meta} =
        Enum.find(params, fn {path, _meta} -> path == [:controller, :pid, :ki] end)

      assert meta.value == 0.0
      assert meta.type == :float
      assert meta.doc == "Integral gain"
      assert meta.default == 0.0
    end
  end

  describe "pubsub notifications" do
    test "publishes change notification when parameter set" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      BB.PubSub.subscribe(TestRobot, [:param, :controller, :pid])

      Parameter.set(TestRobot, [:controller, :pid, :kp], 2.5)

      assert_receive {:bb, [:param, :controller, :pid, :kp],
                      %BB.Message{
                        payload: %ParameterChanged{
                          path: [:controller, :pid, :kp],
                          old_value: nil,
                          new_value: 2.5,
                          source: :local
                        }
                      }}
    end

    test "publishes init notification when defaults set" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:param])

      Parameter.register(TestRobot, [:controller, :pid], TestController)

      # Should receive notifications for defaults being set
      assert_receive {:bb, [:param, :controller, :pid, :ki],
                      %BB.Message{payload: %ParameterChanged{source: :init}}}

      assert_receive {:bb, [:param, :controller, :pid, :kd],
                      %BB.Message{payload: %ParameterChanged{source: :init}}}
    end

    test "includes old value in change notification" do
      start_supervised!(TestRobot)

      Parameter.register(TestRobot, [:controller, :pid], TestController)
      Parameter.set(TestRobot, [:controller, :pid, :kp], 1.0)

      BB.PubSub.subscribe(TestRobot, [:param, :controller, :pid, :kp])

      Parameter.set(TestRobot, [:controller, :pid, :kp], 2.0)

      assert_receive {:bb, [:param, :controller, :pid, :kp],
                      %BB.Message{
                        payload: %ParameterChanged{
                          old_value: 1.0,
                          new_value: 2.0
                        }
                      }}
    end
  end
end
