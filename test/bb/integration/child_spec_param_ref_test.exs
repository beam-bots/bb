# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Integration.ChildSpecParamRefTest do
  @moduledoc """
  Integration tests for parameter references in actuator/sensor/controller options.

  Tests the full flow: DSL definition → compilation → runtime resolution →
  parameter changes → handle_options callback invocation.
  """
  use ExUnit.Case, async: false

  import BB.Unit

  alias BB.Cldr.Unit, as: CldrUnit
  alias BB.{Message, PubSub}
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot.{Runtime, Units}
  alias BB.Robot.State, as: RobotState

  # Use an Agent to track actuator/sensor state changes across process boundaries
  defmodule StateTracker do
    use Agent

    def start_link(_opts), do: Agent.start_link(fn -> [] end, name: __MODULE__)
    def record(event), do: Agent.update(__MODULE__, fn events -> [event | events] end)
    def get_events, do: Agent.get(__MODULE__, & &1)
    def clear, do: Agent.update(__MODULE__, fn _ -> [] end)
  end

  defmodule ParamTrackingActuator do
    @moduledoc """
    Test actuator that tracks param resolution and handle_options calls.
    """
    use BB.Actuator,
      options_schema: [
        max_effort: [type: {:custom, __MODULE__, :validate_unit, []}, required: false]
      ]

    def validate_unit(value), do: {:ok, value}

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    @impl BB.Actuator
    def init(opts) do
      max_effort = Keyword.get(opts, :max_effort)
      StateTracker.record({:actuator_init, max_effort})
      {:ok, %{max_effort: max_effort}}
    end

    @impl BB.Actuator
    def handle_options(new_opts, state) do
      new_max_effort = Keyword.get(new_opts, :max_effort)
      StateTracker.record({:actuator_handle_options, new_max_effort})
      {:ok, %{state | max_effort: new_max_effort}}
    end
  end

  defmodule ParamTrackingSensor do
    @moduledoc """
    Test sensor that tracks param resolution and handle_options calls.
    """
    use BB.Sensor,
      options_schema: [
        sample_rate: [type: {:custom, __MODULE__, :validate_unit, []}, required: false]
      ]

    def validate_unit(value), do: {:ok, value}

    @impl BB.Sensor
    def init(opts) do
      sample_rate = Keyword.get(opts, :sample_rate)
      StateTracker.record({:sensor_init, sample_rate})
      {:ok, %{sample_rate: sample_rate}}
    end

    @impl BB.Sensor
    def handle_options(new_opts, state) do
      new_sample_rate = Keyword.get(new_opts, :sample_rate)
      StateTracker.record({:sensor_handle_options, new_sample_rate})
      {:ok, %{state | sample_rate: new_sample_rate}}
    end
  end

  defmodule ParamTrackingController do
    @moduledoc """
    Test controller that tracks param resolution and handle_options calls.
    """
    use BB.Controller,
      options_schema: [
        gain: [type: :float, required: false]
      ]

    @impl BB.Controller
    def init(opts) do
      gain = Keyword.get(opts, :gain)
      StateTracker.record({:controller_init, gain})
      {:ok, %{gain: gain}}
    end

    @impl BB.Controller
    def handle_options(new_opts, state) do
      new_gain = Keyword.get(new_opts, :gain)
      StateTracker.record({:controller_handle_options, new_gain})
      {:ok, %{state | gain: new_gain}}
    end
  end

  defmodule ParamActuatorRobot do
    @moduledoc false
    use BB

    parameters do
      group :motion do
        param :max_effort,
          type: {:unit, :newton_meter},
          default: ~u(10 newton_meter),
          doc: "Maximum effort for actuators"
      end
    end

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end
    end

    topology do
      link :base do
        joint :shoulder do
          type :revolute

          origin do
            z(~u(0.1 meter))
          end

          axis do
          end

          limit do
            lower(~u(-90 degree))
            upper(~u(90 degree))
            velocity(~u(1 radian_per_second))
            effort(~u(10 newton_meter))
          end

          actuator :motor,
                   {ParamTrackingActuator, max_effort: param([:motion, :max_effort])}

          link :arm do
          end
        end
      end
    end
  end

  defmodule ParamSensorRobot do
    @moduledoc false
    use BB

    parameters do
      group :sensors do
        param :sample_rate,
          type: {:unit, :hertz},
          default: ~u(100 hertz),
          doc: "Sensor sample rate"
      end
    end

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end
    end

    topology do
      link :base do
        sensor :imu,
               {ParamTrackingSensor, sample_rate: param([:sensors, :sample_rate])}
      end
    end
  end

  defmodule ParamControllerRobot do
    @moduledoc false
    use BB

    parameters do
      group :control do
        param :gain,
          type: :float,
          default: 1.5,
          doc: "Controller gain"
      end
    end

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end
    end

    topology do
      link :base do
      end
    end

    controllers do
      controller :position,
                 {ParamTrackingController, gain: param([:control, :gain])}
    end
  end

  setup do
    start_supervised!(StateTracker)
    :ok
  end

  describe "actuator param refs" do
    test "param is resolved at startup" do
      start_supervised!(ParamActuatorRobot)

      # Give time for actuator to initialize
      Process.sleep(100)

      events = StateTracker.get_events()
      init_event = Enum.find(events, fn {type, _} -> type == :actuator_init end)

      assert init_event != nil
      {:actuator_init, max_effort} = init_event

      # Should receive the resolved unit value (10 newton_meter as Cldr.Unit)
      assert max_effort != nil
      assert {:ok, converted} = CldrUnit.convert(max_effort, :newton_meter)
      assert_in_delta Units.extract_float(converted), 10.0, 0.001
    end

    test "handle_options is called when param changes" do
      start_supervised!(ParamActuatorRobot)

      # Wait for init
      Process.sleep(100)
      StateTracker.clear()

      # Change the parameter
      robot_state = Runtime.get_robot_state(ParamActuatorRobot)
      :ok = RobotState.set_parameter(robot_state, [:motion, :max_effort], ~u(20 newton_meter))

      # Publish the change
      message =
        Message.new!(ParameterChanged, :parameter,
          path: [:motion, :max_effort],
          old_value: ~u(10 newton_meter),
          new_value: ~u(20 newton_meter),
          source: :local
        )

      PubSub.publish(ParamActuatorRobot, [:param, :motion, :max_effort], message)

      # Wait for handle_options callback
      Process.sleep(100)

      events = StateTracker.get_events()
      handle_opts_event = Enum.find(events, fn {type, _} -> type == :actuator_handle_options end)

      assert handle_opts_event != nil
      {:actuator_handle_options, new_max_effort} = handle_opts_event

      assert new_max_effort != nil
      assert {:ok, converted} = CldrUnit.convert(new_max_effort, :newton_meter)
      assert_in_delta Units.extract_float(converted), 20.0, 0.001
    end
  end

  describe "sensor param refs" do
    test "param is resolved at startup" do
      start_supervised!(ParamSensorRobot)

      # Give time for sensor to initialize
      Process.sleep(100)

      events = StateTracker.get_events()
      init_event = Enum.find(events, fn {type, _} -> type == :sensor_init end)

      assert init_event != nil
      {:sensor_init, sample_rate} = init_event

      # Should receive the resolved unit value (100 hertz as Cldr.Unit)
      assert sample_rate != nil
      assert {:ok, converted} = CldrUnit.convert(sample_rate, :hertz)
      assert_in_delta Units.extract_float(converted), 100.0, 0.001
    end

    test "handle_options is called when param changes" do
      start_supervised!(ParamSensorRobot)

      # Wait for init
      Process.sleep(100)
      StateTracker.clear()

      # Change the parameter
      robot_state = Runtime.get_robot_state(ParamSensorRobot)
      :ok = RobotState.set_parameter(robot_state, [:sensors, :sample_rate], ~u(200 hertz))

      # Publish the change
      message =
        Message.new!(ParameterChanged, :parameter,
          path: [:sensors, :sample_rate],
          old_value: ~u(100 hertz),
          new_value: ~u(200 hertz),
          source: :local
        )

      PubSub.publish(ParamSensorRobot, [:param, :sensors, :sample_rate], message)

      # Wait for handle_options callback
      Process.sleep(100)

      events = StateTracker.get_events()
      handle_opts_event = Enum.find(events, fn {type, _} -> type == :sensor_handle_options end)

      assert handle_opts_event != nil
      {:sensor_handle_options, new_sample_rate} = handle_opts_event

      assert new_sample_rate != nil
      assert {:ok, converted} = CldrUnit.convert(new_sample_rate, :hertz)
      assert_in_delta Units.extract_float(converted), 200.0, 0.001
    end
  end

  describe "controller param refs" do
    test "param is resolved at startup" do
      start_supervised!(ParamControllerRobot)

      # Give time for controller to initialize
      Process.sleep(100)

      events = StateTracker.get_events()
      init_event = Enum.find(events, fn {type, _} -> type == :controller_init end)

      assert init_event != nil
      {:controller_init, gain} = init_event

      # Should receive the resolved float value (1.5)
      assert_in_delta gain, 1.5, 0.001
    end

    test "handle_options is called when param changes" do
      start_supervised!(ParamControllerRobot)

      # Wait for init
      Process.sleep(100)
      StateTracker.clear()

      # Change the parameter
      robot_state = Runtime.get_robot_state(ParamControllerRobot)
      :ok = RobotState.set_parameter(robot_state, [:control, :gain], 2.5)

      # Publish the change
      message =
        Message.new!(ParameterChanged, :parameter,
          path: [:control, :gain],
          old_value: 1.5,
          new_value: 2.5,
          source: :local
        )

      PubSub.publish(ParamControllerRobot, [:param, :control, :gain], message)

      # Wait for handle_options callback
      Process.sleep(100)

      events = StateTracker.get_events()

      handle_opts_event =
        Enum.find(events, fn {type, _} -> type == :controller_handle_options end)

      assert handle_opts_event != nil
      {:controller_handle_options, new_gain} = handle_opts_event

      assert_in_delta new_gain, 2.5, 0.001
    end
  end
end
