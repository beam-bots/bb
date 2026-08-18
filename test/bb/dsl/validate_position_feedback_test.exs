# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidatePositionFeedbackTest do
  # `capture_io(:stderr, …)` redirects a named process, so a robot compiled by
  # another test running alongside would land in the captured output.
  use ExUnit.Case, async: false

  import ExUnit.CaptureIO

  defmodule BlindActuator do
    @moduledoc false
    use BB.Actuator, options_schema: []

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    @impl BB.Actuator
    def init(_opts), do: {:ok, %{}}

    @impl BB.Actuator
    def handle_command(_message, state), do: {:noreply, state}
  end

  defmodule SelfSensingActuator do
    @moduledoc false
    use BB.Actuator, options_schema: []

    @impl BB.Actuator
    def capabilities, do: [:position_feedback]

    @impl BB.Actuator
    def disarm(_opts), do: :ok

    @impl BB.Actuator
    def init(_opts), do: {:ok, %{}}

    @impl BB.Actuator
    def handle_command(_message, state), do: {:noreply, state}
  end

  defp define(topology) do
    capture_io(:stderr, fn ->
      Code.eval_string("""
      defmodule PositionFeedbackTest#{System.unique_integer([:positive])} do
        use BB

        #{topology}
      end
      """)
    end)
  end

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

  @blind inspect(BlindActuator)
  @self_sensing inspect(SelfSensingActuator)

  test "warns when a joint is driven but nothing reports where it is" do
    warning = define(revolute_joint("actuator :motor, #{@blind}"))

    assert warning =~ "joint :shoulder is driven by :motor but nothing reports where it is"
    assert warning =~ "OpenLoopPositionEstimator"
    assert warning =~ "def capabilities, do: [:position_feedback]"
  end

  test "stays quiet when the joint has a sensor" do
    warning =
      define(
        revolute_joint(
          "actuator :motor, #{@blind}",
          "sensor :position, {BB.Sensor.OpenLoopPositionEstimator, actuator: :motor}"
        )
      )

    refute warning =~ "nothing reports where it is"
  end

  test "stays quiet when the driver reports its own position" do
    warning = define(revolute_joint("actuator :motor, #{@self_sensing}"))

    refute warning =~ "nothing reports where it is"
  end

  test "names every unsensed actuator on the joint" do
    warning =
      define(
        revolute_joint("""
        actuator :left, #{@blind}
        actuator :right, #{@blind}
        actuator :sensing, #{@self_sensing}
        """)
      )

    assert warning =~ "driven by :left, :right but nothing reports"
  end

  test "stays quiet about a joint with no actuator to drive it" do
    warning = define(revolute_joint(""))

    refute warning =~ "nothing reports where it is"
  end

  test "stays quiet about a fixed joint, which has nowhere to go" do
    warning =
      define("""
      topology do
        link :base do
          joint :mount do
            type :fixed

            actuator :motor, #{@blind}

            link :arm
          end
        end
      end
      """)

    refute warning =~ "nothing reports where it is"
  end
end
