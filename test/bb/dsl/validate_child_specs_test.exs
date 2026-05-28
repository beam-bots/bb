# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidateChildSpecsTest do
  use ExUnit.Case, async: true

  # A bare GenServer that does NOT declare any BB component behaviour. The
  # verifier should refuse to wire this in as a sensor / actuator / etc.
  defmodule NotAComponent do
    @moduledoc false
    use GenServer

    @impl GenServer
    def init(opts), do: {:ok, opts}
  end

  describe "behaviour enforcement" do
    test "actuator module without @behaviour BB.Actuator raises DslError" do
      assert_raise Spark.Error.DslError, ~r/must implement the BB\.Actuator behaviour/, fn ->
        defmodule ActuatorBadRobot do
          @moduledoc false
          use BB

          topology do
            link :base do
              joint :shoulder do
                type :revolute

                limit do
                  effort(~u(10 newton_meter))
                  velocity(~u(180 degree_per_second))
                end

                actuator :motor, BB.Dsl.Verifiers.ValidateChildSpecsTest.NotAComponent

                link :arm
              end
            end
          end
        end
      end
    end

    test "link sensor module without @behaviour BB.Sensor raises DslError" do
      assert_raise Spark.Error.DslError, ~r/must implement the BB\.Sensor behaviour/, fn ->
        defmodule SensorBadRobot do
          @moduledoc false
          use BB

          topology do
            link :base do
              sensor :imu, BB.Dsl.Verifiers.ValidateChildSpecsTest.NotAComponent
            end
          end
        end
      end
    end

    test "robot-level controller without @behaviour BB.Controller raises DslError" do
      assert_raise Spark.Error.DslError, ~r/must implement the BB\.Controller behaviour/, fn ->
        defmodule ControllerBadRobot do
          @moduledoc false
          use BB

          controllers do
            controller(:bad, BB.Dsl.Verifiers.ValidateChildSpecsTest.NotAComponent)
          end

          topology do
            link :base
          end
        end
      end
    end
  end
end
