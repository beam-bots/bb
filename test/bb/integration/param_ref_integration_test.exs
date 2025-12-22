# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Integration.ParamRefIntegrationTest do
  @moduledoc """
  Integration tests for parameter references in DSL.

  Tests the full flow: DSL definition → compilation → runtime resolution → updates.
  """
  use ExUnit.Case, async: true

  import BB.Unit

  alias BB.{Message, PubSub}
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  defmodule ParamRefRobot do
    @moduledoc false
    use BB

    parameters do
      group :motion do
        param :max_effort,
          type: {:unit, :newton_meter},
          default: ~u(10 newton_meter),
          doc: "Maximum effort for joints"

        param :max_velocity,
          type: {:unit, :radian_per_second},
          default: ~u(2 radian_per_second),
          doc: "Maximum velocity for joints"
      end

      group :limits do
        param :shoulder_lower,
          type: {:unit, :degree},
          default: ~u(-90 degree),
          doc: "Lower limit for shoulder"

        param :shoulder_upper,
          type: {:unit, :degree},
          default: ~u(90 degree),
          doc: "Upper limit for shoulder"
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
            lower(param([:limits, :shoulder_lower]))
            upper(param([:limits, :shoulder_upper]))
            effort(param([:motion, :max_effort]))
            velocity(param([:motion, :max_velocity]))
          end

          link :arm do
          end
        end
      end
    end
  end

  describe "param ref compilation" do
    test "robot compiles successfully with param refs" do
      # If we get here, compilation succeeded
      assert ParamRefRobot.robot() != nil
    end

    test "robot has param_subscriptions" do
      robot = ParamRefRobot.robot()

      assert map_size(robot.param_subscriptions) > 0
    end

    test "param_subscriptions maps parameter paths to joint locations" do
      robot = ParamRefRobot.robot()

      # Check that limits:shoulder_lower is tracked
      assert Map.has_key?(robot.param_subscriptions, [:limits, :shoulder_lower])

      # Check that motion:max_effort is tracked
      assert Map.has_key?(robot.param_subscriptions, [:motion, :max_effort])
    end
  end

  describe "param ref resolution at startup" do
    test "param refs are resolved to current parameter values" do
      start_supervised!(ParamRefRobot)

      robot = Runtime.get_robot(ParamRefRobot)
      joint = robot.joints.shoulder

      # Limits should be resolved from parameter defaults
      # Default max_effort is 10 newton_meter ≈ 10.0 in SI
      assert_in_delta joint.limits.effort, 10.0, 0.001

      # Default max_velocity is 2 rad/s
      assert_in_delta joint.limits.velocity, 2.0, 0.001

      # Default shoulder_lower is -90 degrees ≈ -π/2 radians
      assert_in_delta joint.limits.lower, -:math.pi() / 2, 0.001

      # Default shoulder_upper is 90 degrees ≈ π/2 radians
      assert_in_delta joint.limits.upper, :math.pi() / 2, 0.001
    end
  end

  describe "param ref updates at runtime" do
    test "robot struct updates when parameter changes" do
      start_supervised!(ParamRefRobot)

      # Get initial value
      robot = Runtime.get_robot(ParamRefRobot)
      initial_effort = robot.joints.shoulder.limits.effort
      assert_in_delta initial_effort, 10.0, 0.001

      # Change the parameter
      robot_state = Runtime.get_robot_state(ParamRefRobot)
      :ok = RobotState.set_parameter(robot_state, [:motion, :max_effort], ~u(20 newton_meter))

      # Publish the change
      message =
        Message.new!(ParameterChanged, :parameter,
          path: [:motion, :max_effort],
          old_value: ~u(10 newton_meter),
          new_value: ~u(20 newton_meter),
          source: :local
        )

      PubSub.publish(ParamRefRobot, [:param, :motion, :max_effort], message)

      # Give the Runtime time to process
      Process.sleep(50)

      # Check that the robot struct was updated
      robot = Runtime.get_robot(ParamRefRobot)
      new_effort = robot.joints.shoulder.limits.effort
      assert_in_delta new_effort, 20.0, 0.001
    end
  end
end
