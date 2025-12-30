# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.MoveToTest do
  use ExUnit.Case, async: true

  alias BB.Command.Context
  alias BB.Command.MoveTo
  alias BB.Error.Invalid.Command, as: InvalidCommand
  alias BB.Error.Kinematics.Unreachable
  alias BB.Math.Vec3
  alias BB.Robot.State, as: RobotState
  alias BB.Test.MockSolver

  defmodule MoveToTestRobot do
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
            joint :tip_joint do
              type(:fixed)

              origin do
                x(~u(0.3 meter))
              end

              link(:tip)
            end
          end
        end
      end
    end
  end

  describe "handle_command/2" do
    test "returns error when target is missing in single-target mode" do
      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{target: {0.3, 0.0, 0.0}, solver: MockSolver}

      assert {:error, %InvalidCommand{argument: :target_link, reason: "required"}} =
               MoveTo.handle_command(goal, context)
    end

    test "returns error when solver is missing in single-target mode" do
      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{target: Vec3.new(0.3, 0.0, 0.0), target_link: :tip}

      assert {:error, %InvalidCommand{argument: :solver, reason: "required"}} =
               MoveTo.handle_command(goal, context)
    end

    test "returns ok with metadata on success" do
      start_supervised!(MoveToTestRobot)

      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5},
         %{
           iterations: 10,
           residual: 0.001,
           reached: true
         }}
      )

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{
        target: Vec3.new(0.3, 0.0, 0.0),
        target_link: :tip,
        solver: MockSolver
      }

      {:ok, meta} = MoveTo.handle_command(goal, context)

      assert meta.reached == true
      assert meta.iterations == 10
    end

    test "returns error on solver failure" do
      start_supervised!(MoveToTestRobot)

      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:error, MockSolver.unreachable_error(:tip, iterations: 50, residual: 0.5)}
      )

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{
        target: Vec3.new(10.0, 0.0, 0.0),
        target_link: :tip,
        solver: MockSolver
      }

      {:error, %Unreachable{} = error} = MoveTo.handle_command(goal, context)

      assert error.iterations == 50
      assert error.residual == 0.5
    end

    test "passes solver options through" do
      start_supervised!(MoveToTestRobot)

      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result({:ok, %{}, %{iterations: 1, residual: 0.0, reached: true}})

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{
        target: Vec3.new(0.3, 0.0, 0.0),
        target_link: :tip,
        solver: MockSolver,
        max_iterations: 100,
        tolerance: 0.01,
        respect_limits: false
      }

      {:ok, _meta} = MoveTo.handle_command(goal, context)

      {_robot, _state, _link, _target, opts} = MockSolver.last_call()
      assert opts[:max_iterations] == 100
      assert opts[:tolerance] == 0.01
      assert opts[:respect_limits] == false
    end

    test "handles multi-target mode" do
      start_supervised!(MoveToTestRobot)

      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      MockSolver.set_result(
        {:ok, %{shoulder_joint: 0.5}, %{iterations: 10, residual: 0.001, reached: true}}
      )

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{
        targets: %{tip: {0.3, 0.0, 0.0}},
        solver: MockSolver
      }

      {:ok, results} = MoveTo.handle_command(goal, context)

      assert Map.has_key?(results, :tip)
      assert {:ok, _positions, _meta} = results[:tip]
    end

    test "returns error when targets missing solver" do
      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{targets: %{tip: {0.3, 0.0, 0.0}}}

      assert {:error, %InvalidCommand{argument: :solver, reason: "required"}} =
               MoveTo.handle_command(goal, context)
    end

    test "returns error when neither target nor targets provided" do
      robot = MoveToTestRobot.robot()
      {:ok, robot_state} = RobotState.new(robot)

      context = %Context{
        robot_module: MoveToTestRobot,
        robot: robot,
        robot_state: robot_state,
        execution_id: make_ref()
      }

      goal = %{solver: MockSolver}

      assert {:error,
              %InvalidCommand{
                argument: :target_or_targets,
                reason: "required: must specify either :target or :targets"
              }} =
               MoveTo.handle_command(goal, context)
    end
  end
end
