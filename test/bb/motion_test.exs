# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.MotionTest do
  use ExUnit.Case, async: true

  alias BB.Command.Context
  alias BB.Error.Kinematics.Unreachable
  alias BB.Motion
  alias BB.Robot.State, as: RobotState
  alias BB.Test.MockSolver

  defmodule MotionTestRobot do
    @moduledoc false
    use BB
    import BB.Unit

    topology do
      link :base_link do
        joint :shoulder_joint do
          type(:revolute)

          axis do
          end

          limit do
            lower(~u(-90 degree))
            upper(~u(90 degree))
            effort(~u(10 newton_meter))
            velocity(~u(90 degree_per_second))
          end

          actuator :shoulder_servo, BB.Test.MockActuator

          link :upper_arm do
            joint :elbow_joint do
              type(:revolute)

              origin do
                x(~u(0.3 meter))
              end

              axis do
              end

              limit do
                lower(~u(-90 degree))
                upper(~u(90 degree))
                effort(~u(5 newton_meter))
                velocity(~u(90 degree_per_second))
              end

              actuator :elbow_servo, BB.Test.MockActuator

              link :forearm do
                joint :tip_joint do
                  type(:fixed)

                  origin do
                    x(~u(0.2 meter))
                  end

                  link(:tip)
                end
              end
            end
          end
        end
      end
    end
  end

  describe "solve_only/4" do
    test "calls solver with correct arguments" do
      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)
      target = {0.3, 0.2, 0.1}

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5, elbow_joint: 0.3},
         %{
           iterations: 10,
           residual: 0.001,
           reached: true
         }}
      )

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      {:ok, positions, meta} = Motion.solve_only(context, :tip, target, solver: MockSolver)

      assert positions == %{shoulder_joint: 0.5, elbow_joint: 0.3}
      assert meta.iterations == 10
      assert meta.residual == 0.001
      assert meta.reached == true

      {called_robot, _called_state, called_link, called_target, _opts} = MockSolver.last_call()
      assert called_robot == robot
      assert called_link == :tip
      assert called_target == target
    end

    test "passes solver options through" do
      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result({:ok, %{}, %{iterations: 1, residual: 0.0, reached: true}})

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      Motion.solve_only(context, :tip, {0.3, 0.2, 0.1},
        solver: MockSolver,
        max_iterations: 100,
        tolerance: 0.01,
        respect_limits: false
      )

      {_robot, _state, _link, _target, opts} = MockSolver.last_call()
      assert opts[:max_iterations] == 100
      assert opts[:tolerance] == 0.01
      assert opts[:respect_limits] == false
    end

    test "returns solver error" do
      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:error, MockSolver.unreachable_error(:tip, iterations: 50, residual: 0.5)}
      )

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      {:error, %Unreachable{} = error} =
        Motion.solve_only(context, :tip, {10.0, 0.0, 0.0}, solver: MockSolver)

      assert error.iterations == 50
      assert error.residual == 0.5
    end
  end

  describe "move_to/4" do
    test "updates robot state on success" do
      start_supervised!(MotionTestRobot)

      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5, elbow_joint: 0.3},
         %{
           iterations: 10,
           residual: 0.001,
           reached: true
         }}
      )

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      {:ok, meta} = Motion.move_to(context, :tip, {0.3, 0.2, 0.1}, solver: MockSolver)

      assert meta.reached == true

      assert RobotState.get_joint_position(robot_state, :shoulder_joint) == 0.5
      assert RobotState.get_joint_position(robot_state, :elbow_joint) == 0.3
    end

    test "does not update state on error" do
      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      RobotState.set_joint_position(robot_state, :shoulder_joint, 1.0)
      RobotState.set_joint_position(robot_state, :elbow_joint, 1.0)

      MockSolver.set_result(
        {:error, MockSolver.unreachable_error(:tip, iterations: 50, residual: 0.5)}
      )

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      {:error, %Unreachable{}} =
        Motion.move_to(context, :tip, {10.0, 0.0, 0.0}, solver: MockSolver)

      assert RobotState.get_joint_position(robot_state, :shoulder_joint) == 1.0
      assert RobotState.get_joint_position(robot_state, :elbow_joint) == 1.0
    end
  end

  describe "send_positions/3" do
    test "updates robot state" do
      start_supervised!(MotionTestRobot)

      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      positions = %{shoulder_joint: 0.7, elbow_joint: 0.4}
      :ok = Motion.send_positions(context, positions)

      assert RobotState.get_joint_position(robot_state, :shoulder_joint) == 0.7
      assert RobotState.get_joint_position(robot_state, :elbow_joint) == 0.4
    end
  end

  describe "with runtime" do
    test "accepts robot module and fetches robot/state from runtime" do
      start_supervised!(MotionTestRobot)

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5, elbow_joint: 0.3},
         %{
           iterations: 10,
           residual: 0.001,
           reached: true
         }}
      )

      {:ok, _positions, meta} =
        Motion.solve_only(MotionTestRobot, :tip, {0.3, 0.2, 0.1}, solver: MockSolver)

      assert meta.reached == true

      {called_robot, _state, _link, _target, _opts} = MockSolver.last_call()
      assert called_robot.name == MotionTestRobot
    end
  end

  describe "solve_only_multi/3" do
    test "solves multiple targets" do
      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5}, %{iterations: 10, residual: 0.001, reached: true}}
      )

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      targets = %{tip: {0.3, 0.2, 0.1}, upper_arm: {0.2, 0.1, 0.0}}

      {:ok, results} = Motion.solve_only_multi(context, targets, solver: MockSolver)

      assert Map.has_key?(results, :tip)
      assert Map.has_key?(results, :upper_arm)
      assert {:ok, _positions, _meta} = results[:tip]
    end

    test "returns error on first failure" do
      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      targets = %{tip: {0.3, 0.2, 0.1}, upper_arm: {10.0, 0.0, 0.0}}

      defmodule FailOnSecondSolver do
        @behaviour BB.IK.Solver

        alias BB.Error.Kinematics.Unreachable

        def solve(_robot, _state, target_link, _target, _opts) do
          if target_link == :upper_arm do
            {:error,
             %Unreachable{
               target_link: :upper_arm,
               iterations: 50,
               residual: 0.5,
               reason: "Target beyond workspace"
             }}
          else
            {:ok, %{shoulder_joint: 0.5}, %{iterations: 10, residual: 0.001, reached: true}}
          end
        end
      end

      {:error, :upper_arm, %Unreachable{}, results} =
        Motion.solve_only_multi(context, targets, solver: FailOnSecondSolver)

      assert {:error, %Unreachable{}} = results[:upper_arm]
    end
  end

  describe "move_to_multi/3" do
    test "moves to multiple targets" do
      start_supervised!(MotionTestRobot)

      robot = MotionTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5, elbow_joint: 0.3},
         %{iterations: 10, residual: 0.001, reached: true}}
      )

      context = %Context{
        robot_module: MotionTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      targets = %{tip: {0.3, 0.2, 0.1}}

      {:ok, results} = Motion.move_to_multi(context, targets, solver: MockSolver)

      assert {:ok, _positions, meta} = results[:tip]
      assert meta.reached == true
    end
  end
end
