# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ParameterTest do
  use ExUnit.Case, async: true
  import BB.Unit

  alias BB.Parameter
  alias BB.Parameter.Changed, as: ParameterChanged

  defmodule RobotWithParameters do
    @moduledoc false
    use BB

    parameters do
      group :motion do
        param :max_speed, type: :float, default: 1.0, doc: "Maximum speed in m/s"
        param :acceleration, type: :float, default: 0.5
      end

      param :debug_mode, type: :boolean, default: false
    end

    topology do
      link :base_link do
      end
    end
  end

  defmodule RobotWithNestedGroups do
    @moduledoc false
    use BB

    parameters do
      group :controller do
        group :pid do
          param :kp, type: :float, default: 1.0
          param :ki, type: :float, default: 0.1
          param :kd, type: :float, default: 0.01
        end
      end
    end

    topology do
      link :base_link do
      end
    end
  end

  defmodule RobotWithNoParameters do
    @moduledoc false
    use BB

    topology do
      link :base_link do
      end
    end
  end

  defmodule TestSensor do
    @moduledoc false
    use BB.Sensor

    def start_link(opts), do: GenServer.start_link(__MODULE__, opts)

    @impl GenServer
    def init(opts), do: {:ok, opts}
  end

  defmodule TestActuator do
    @moduledoc false
    use BB.Actuator

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    def start_link(opts), do: GenServer.start_link(__MODULE__, opts)

    @impl GenServer
    def init(opts), do: {:ok, opts}
  end

  defmodule TestController do
    @moduledoc false
    use BB.Controller

    def start_link(opts), do: GenServer.start_link(__MODULE__, opts)

    @impl GenServer
    def init(opts), do: {:ok, opts}
  end

  defmodule RobotWithComponentParams do
    @moduledoc false
    use BB

    controllers do
      controller :pid_ctrl, TestController do
        param :kp, type: :float, default: 1.0
        param :ki, type: :float, default: 0.1
      end
    end

    sensors do
      sensor :gps, TestSensor do
        param :update_rate, type: :integer, default: 10
      end
    end

    topology do
      link :base_link do
        sensor :imu, TestSensor do
          param :sample_rate, type: :integer, default: 100
        end

        joint :shoulder do
          type(:revolute)

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(1 radian_per_second))
          end

          actuator :motor, TestActuator do
            param :max_torque, type: :float, default: 5.0
          end

          link :upper_arm do
          end
        end
      end
    end
  end

  defmodule RobotWithUnitParams do
    @moduledoc false
    use BB

    parameters do
      group :motion do
        param :max_speed, type: {:unit, :meter_per_second}, default: ~u(1.5 meter_per_second)

        param :max_angular_speed,
          type: {:unit, :radian_per_second},
          default: ~u(0.5 radian_per_second)
      end

      param :arm_length, type: {:unit, :meter}, default: ~u(0.5 meter)
    end

    topology do
      link :base_link do
      end
    end
  end

  describe "generated functions" do
    test "robot module has __bb_parameter_schema__/0" do
      assert function_exported?(RobotWithParameters, :__bb_parameter_schema__, 0)
    end

    test "robot module has __bb_default_parameters__/0" do
      assert function_exported?(RobotWithParameters, :__bb_default_parameters__, 0)
    end

    test "__bb_parameter_schema__ returns schema list" do
      schema = RobotWithParameters.__bb_parameter_schema__()

      assert is_list(schema)

      # Check motion group parameters
      assert {[:motion, :max_speed], opts} =
               Enum.find(schema, fn {path, _} -> path == [:motion, :max_speed] end)

      assert Keyword.get(opts, :type) == :float
      assert Keyword.get(opts, :doc) == "Maximum speed in m/s"

      # Check root-level parameter
      assert {[:debug_mode], _opts} = Enum.find(schema, fn {path, _} -> path == [:debug_mode] end)
    end

    test "__bb_default_parameters__ returns default values" do
      defaults = RobotWithParameters.__bb_default_parameters__()

      assert is_list(defaults)
      assert {[:motion, :max_speed], 1.0} in defaults
      assert {[:motion, :acceleration], 0.5} in defaults
      assert {[:debug_mode], false} in defaults
    end

    test "nested groups produce correct paths" do
      schema = RobotWithNestedGroups.__bb_parameter_schema__()

      paths = Enum.map(schema, fn {path, _} -> path end)

      assert [:controller, :pid, :kp] in paths
      assert [:controller, :pid, :ki] in paths
      assert [:controller, :pid, :kd] in paths
    end

    test "robot with no parameters has empty functions" do
      assert RobotWithNoParameters.__bb_parameter_schema__() == []
      assert RobotWithNoParameters.__bb_default_parameters__() == []
    end
  end

  describe "runtime registration" do
    test "DSL parameters are registered at startup" do
      start_supervised!(RobotWithParameters)

      # Parameters should be accessible
      assert {:ok, value} = Parameter.get(RobotWithParameters, [:motion, :max_speed])
      assert value == 1.0

      assert {:ok, value} = Parameter.get(RobotWithParameters, [:motion, :acceleration])
      assert value == 0.5

      assert {:ok, value} = Parameter.get(RobotWithParameters, [:debug_mode])
      assert value == false
    end

    test "nested group parameters are registered correctly" do
      start_supervised!(RobotWithNestedGroups)

      assert {:ok, value} = Parameter.get(RobotWithNestedGroups, [:controller, :pid, :kp])
      assert value == 1.0

      assert {:ok, value} = Parameter.get(RobotWithNestedGroups, [:controller, :pid, :ki])
      assert value == 0.1
    end

    test "DSL parameters can be set" do
      start_supervised!(RobotWithParameters)

      assert :ok = Parameter.set(RobotWithParameters, [:motion, :max_speed], 2.0)
      assert {:ok, 2.0} = Parameter.get(RobotWithParameters, [:motion, :max_speed])
    end

    test "DSL parameters are validated" do
      start_supervised!(RobotWithParameters)

      # Type mismatch - max_speed expects float
      assert {:error, _} =
               Parameter.set(RobotWithParameters, [:motion, :max_speed], "not a float")
    end

    test "unknown root-level parameters are rejected" do
      start_supervised!(RobotWithParameters)

      # Root-level schema exists (for debug_mode), so this is an unknown param, not unregistered
      assert {:error, {:unknown_parameter, :nonexistent}} =
               Parameter.set(RobotWithParameters, [:nonexistent], 1.0)
    end

    test "parameters under unregistered groups are rejected" do
      start_supervised!(RobotWithParameters)

      # No schema registered for [:totally_fake], so this is unregistered
      assert {:error, {:unregistered_parameter, [:totally_fake, :param]}} =
               Parameter.set(RobotWithParameters, [:totally_fake, :param], 1.0)
    end

    test "unknown parameters within group are rejected" do
      start_supervised!(RobotWithParameters)

      assert {:error, {:unknown_parameter, :nonexistent}} =
               Parameter.set(RobotWithParameters, [:motion, :nonexistent], 1.0)
    end
  end

  describe "pubsub notifications" do
    test "defaults are applied at startup" do
      start_supervised!(RobotWithParameters)

      # Verify defaults were applied (init notifications can't be captured
      # since we can't subscribe before the registry exists)
      assert {:ok, 1.0} = Parameter.get(RobotWithParameters, [:motion, :max_speed])
      assert {:ok, 0.5} = Parameter.get(RobotWithParameters, [:motion, :acceleration])
      assert {:ok, false} = Parameter.get(RobotWithParameters, [:debug_mode])
    end

    test "change notifications include old and new values" do
      start_supervised!(RobotWithParameters)

      BB.PubSub.subscribe(RobotWithParameters, [:param, :motion, :max_speed])

      Parameter.set(RobotWithParameters, [:motion, :max_speed], 3.0)

      assert_receive {:bb, [:param, :motion, :max_speed],
                      %BB.Message{
                        payload: %ParameterChanged{
                          old_value: 1.0,
                          new_value: 3.0,
                          source: :local
                        }
                      }}
    end
  end

  describe "component inline params" do
    test "controller params are collected" do
      schema = RobotWithComponentParams.__bb_parameter_schema__()
      paths = Enum.map(schema, fn {path, _} -> path end)

      assert [:controller, :pid_ctrl, :kp] in paths
      assert [:controller, :pid_ctrl, :ki] in paths
    end

    test "robot-level sensor params are collected" do
      schema = RobotWithComponentParams.__bb_parameter_schema__()
      paths = Enum.map(schema, fn {path, _} -> path end)

      assert [:sensor, :gps, :update_rate] in paths
    end

    test "topology sensor params are collected" do
      schema = RobotWithComponentParams.__bb_parameter_schema__()
      paths = Enum.map(schema, fn {path, _} -> path end)

      assert [:link, :base_link, :sensor, :imu, :sample_rate] in paths
    end

    test "topology actuator params are collected" do
      schema = RobotWithComponentParams.__bb_parameter_schema__()
      paths = Enum.map(schema, fn {path, _} -> path end)

      assert [:link, :base_link, :joint, :shoulder, :actuator, :motor, :max_torque] in paths
    end

    test "component params have defaults" do
      defaults = RobotWithComponentParams.__bb_default_parameters__()

      assert {[:controller, :pid_ctrl, :kp], 1.0} in defaults
      assert {[:controller, :pid_ctrl, :ki], 0.1} in defaults
      assert {[:sensor, :gps, :update_rate], 10} in defaults
    end

    test "component params are registered at runtime" do
      start_supervised!(RobotWithComponentParams)

      assert {:ok, 1.0} = Parameter.get(RobotWithComponentParams, [:controller, :pid_ctrl, :kp])
      assert {:ok, 10} = Parameter.get(RobotWithComponentParams, [:sensor, :gps, :update_rate])

      assert {:ok, 100} =
               Parameter.get(RobotWithComponentParams, [
                 :link,
                 :base_link,
                 :sensor,
                 :imu,
                 :sample_rate
               ])

      assert {:ok, 5.0} =
               Parameter.get(
                 RobotWithComponentParams,
                 [:link, :base_link, :joint, :shoulder, :actuator, :motor, :max_torque]
               )
    end
  end

  describe "parameter listing" do
    test "list returns all DSL parameters with metadata" do
      start_supervised!(RobotWithParameters)

      params = Parameter.list(RobotWithParameters)

      # Should include all parameters with their values
      paths = Enum.map(params, fn {path, _meta} -> path end)

      assert [:motion, :max_speed] in paths
      assert [:motion, :acceleration] in paths
      assert [:debug_mode] in paths

      # Check metadata
      {_, meta} = Enum.find(params, fn {path, _} -> path == [:motion, :max_speed] end)
      assert meta.value == 1.0
      assert meta.type == :float
      assert meta.doc == "Maximum speed in m/s"
    end

    test "list with prefix filters parameters" do
      start_supervised!(RobotWithParameters)

      params = Parameter.list(RobotWithParameters, prefix: [:motion])
      paths = Enum.map(params, fn {path, _meta} -> path end)

      assert [:motion, :max_speed] in paths
      assert [:motion, :acceleration] in paths
      refute [:debug_mode] in paths
    end
  end

  describe "unit-typed parameters" do
    test "unit params have unit_type schema" do
      schema = RobotWithUnitParams.__bb_parameter_schema__()

      {[:motion, :max_speed], opts} =
        Enum.find(schema, fn {path, _} -> path == [:motion, :max_speed] end)

      # Type should be the custom unit validation tuple
      assert {:custom, BB.Unit.Option, :validate, [opts_list]} = Keyword.get(opts, :type)
      assert Keyword.get(opts_list, :compatible) == :meter_per_second
    end

    test "unit params have unit defaults" do
      defaults = RobotWithUnitParams.__bb_default_parameters__()

      {[:motion, :max_speed], default} =
        Enum.find(defaults, fn {path, _} -> path == [:motion, :max_speed] end)

      assert %Cldr.Unit{} = default
      assert default.unit == :meter_per_second
    end

    test "unit params are registered at runtime" do
      start_supervised!(RobotWithUnitParams)

      assert {:ok, value} = Parameter.get(RobotWithUnitParams, [:motion, :max_speed])
      assert %Cldr.Unit{} = value
      assert value.unit == :meter_per_second
    end

    test "unit params can be set with compatible units" do
      start_supervised!(RobotWithUnitParams)

      # Set with same unit
      assert :ok =
               Parameter.set(RobotWithUnitParams, [:motion, :max_speed], ~u(2.0 meter_per_second))

      assert {:ok, value} = Parameter.get(RobotWithUnitParams, [:motion, :max_speed])
      assert Cldr.Unit.compare(value, ~u(2.0 meter_per_second)) == :eq

      # Set with compatible unit (should work - kilometer_per_hour is compatible with meter_per_second)
      assert :ok =
               Parameter.set(
                 RobotWithUnitParams,
                 [:motion, :max_speed],
                 ~u(36 kilometer_per_hour)
               )

      assert {:ok, value} = Parameter.get(RobotWithUnitParams, [:motion, :max_speed])
      assert Cldr.Unit.compare(value, ~u(10 meter_per_second)) == :eq
    end

    test "unit params reject incompatible units" do
      start_supervised!(RobotWithUnitParams)

      # Try to set a length (meter) as a velocity - should fail
      assert {:error, _} =
               Parameter.set(RobotWithUnitParams, [:motion, :max_speed], ~u(1.0 meter))
    end

    test "unit params reject non-unit values" do
      start_supervised!(RobotWithUnitParams)

      assert {:error, _} = Parameter.set(RobotWithUnitParams, [:motion, :max_speed], 1.5)
      assert {:error, _} = Parameter.set(RobotWithUnitParams, [:motion, :max_speed], "fast")
    end
  end

  describe "start_link params" do
    test "params override defaults" do
      start_supervised!({RobotWithParameters, params: [motion: [max_speed: 5.0]]})

      assert {:ok, 5.0} = Parameter.get(RobotWithParameters, [:motion, :max_speed])
      assert {:ok, 0.5} = Parameter.get(RobotWithParameters, [:motion, :acceleration])
    end

    test "multiple params can be set" do
      start_supervised!(
        {RobotWithParameters,
         params: [motion: [max_speed: 5.0, acceleration: 2.0], debug_mode: true]}
      )

      assert {:ok, 5.0} = Parameter.get(RobotWithParameters, [:motion, :max_speed])
      assert {:ok, 2.0} = Parameter.get(RobotWithParameters, [:motion, :acceleration])
      assert {:ok, true} = Parameter.get(RobotWithParameters, [:debug_mode])
    end

    test "nested group params work" do
      start_supervised!({RobotWithNestedGroups, params: [controller: [pid: [kp: 2.5, ki: 0.2]]]})

      assert {:ok, 2.5} = Parameter.get(RobotWithNestedGroups, [:controller, :pid, :kp])
      assert {:ok, 0.2} = Parameter.get(RobotWithNestedGroups, [:controller, :pid, :ki])
      assert {:ok, 0.01} = Parameter.get(RobotWithNestedGroups, [:controller, :pid, :kd])
    end

    test "empty params is valid" do
      start_supervised!({RobotWithParameters, params: []})

      assert {:ok, 1.0} = Parameter.get(RobotWithParameters, [:motion, :max_speed])
    end

    test "unknown params cause startup failure" do
      assert {:error, {{:shutdown, {:failed_to_start_child, _, reason}}, _}} =
               start_supervised({RobotWithParameters, params: [unknown_param: 42]})

      assert %Spark.Options.ValidationError{} = reason
    end

    test "unknown nested params cause startup failure" do
      assert {:error, {{:shutdown, {:failed_to_start_child, _, reason}}, _}} =
               start_supervised({RobotWithParameters, params: [motion: [unknown: 1.0]]})

      assert %Spark.Options.ValidationError{} = reason
    end

    test "type mismatch causes startup failure" do
      assert {:error, {{:shutdown, {:failed_to_start_child, _, reason}}, _}} =
               start_supervised(
                 {RobotWithParameters, params: [motion: [max_speed: "not a float"]]}
               )

      assert %Spark.Options.ValidationError{} = reason
    end

    test "startup params override defaults when setting later" do
      start_supervised!({RobotWithParameters, params: [motion: [max_speed: 7.0]]})

      BB.PubSub.subscribe(RobotWithParameters, [:param, :motion, :max_speed])

      Parameter.set(RobotWithParameters, [:motion, :max_speed], 8.0)

      assert_receive {:bb, [:param, :motion, :max_speed],
                      %BB.Message{
                        payload: %ParameterChanged{
                          old_value: 7.0,
                          new_value: 8.0,
                          source: :local
                        }
                      }}
    end

    test "robot with no parameters accepts empty params" do
      start_supervised!({RobotWithNoParameters, params: []})
    end

    test "component params can be set at startup" do
      start_supervised!(
        {RobotWithComponentParams,
         params: [
           controller: [pid_ctrl: [kp: 3.0]],
           sensor: [gps: [update_rate: 20]]
         ]}
      )

      assert {:ok, 3.0} = Parameter.get(RobotWithComponentParams, [:controller, :pid_ctrl, :kp])
      assert {:ok, 20} = Parameter.get(RobotWithComponentParams, [:sensor, :gps, :update_rate])
    end

    test "unit params can be set at startup" do
      start_supervised!(
        {RobotWithUnitParams, params: [motion: [max_speed: ~u(3.0 meter_per_second)]]}
      )

      assert {:ok, value} = Parameter.get(RobotWithUnitParams, [:motion, :max_speed])
      assert Cldr.Unit.compare(value, ~u(3.0 meter_per_second)) == :eq
    end

    test "unit params with incompatible units cause startup failure" do
      assert {:error, {{:shutdown, {:failed_to_start_child, _, reason}}, _}} =
               start_supervised({RobotWithUnitParams, params: [motion: [max_speed: ~u(1 meter)]]})

      assert %Spark.Options.ValidationError{} = reason
    end
  end
end
