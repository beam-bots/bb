# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidatePositionFeedbackTest do
  # `capture_io(:stderr, …)` redirects a named process, so a robot compiled by
  # another test running alongside would land in the captured output.
  use ExUnit.Case, async: false

  import ExUnit.CaptureIO

  defp define(module_body) do
    capture_io(:stderr, fn ->
      Code.eval_string("""
      defmodule #{unique_module()} do
        use BB

        #{module_body}
      end
      """)
    end)
  end

  defp unique_module, do: "PositionFeedbackTest#{System.unique_integer([:positive])}"

  defp revolute_joint(actuator, sensors \\ "") do
    """
    topology do
      link :base do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          #{actuator}
          #{sensors}

          link :arm
        end
      end
    end
    """
  end

  test "warns when a joint is driven but nothing reports where it is" do
    warning = define(revolute_joint("actuator :motor, BB.Test.MockActuator"))

    assert warning =~ "joint :shoulder is driven by :motor but has no sensor"
    assert warning =~ "OpenLoopPositionEstimator"
    assert warning =~ "sensor: false"
  end

  test "stays quiet when the joint has a sensor" do
    warning =
      define(
        revolute_joint(
          "actuator :motor, BB.Test.MockActuator",
          "sensor :position, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor}"
        )
      )

    refute warning =~ "has no sensor"
  end

  test "stays quiet when the actuator says it doesn't need one" do
    warning = define(revolute_joint("actuator :motor, BB.Test.MockActuator, sensor: false"))

    refute warning =~ "has no sensor"
  end

  test "stays quiet about a joint with no actuator to drive it" do
    warning = define(revolute_joint(""))

    refute warning =~ "has no sensor"
  end

  test "stays quiet about a fixed joint, which has nowhere to go" do
    warning =
      define("""
      topology do
        link :base do
          joint :mount do
            type :fixed

            actuator :motor, BB.Test.MockActuator

            link :arm
          end
        end
      end
      """)

    refute warning =~ "has no sensor"
  end
end
