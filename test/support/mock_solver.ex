# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.MockSolver do
  @moduledoc """
  Mock IK solver for testing BB.Motion without real IK computations.

  Configure behaviour via process dictionary:
  - `:mock_solver_result` - the result to return from solve/5

  ## Examples

      BB.Test.MockSolver.set_result({:ok, %{joint1: 0.5}, %{iterations: 5, residual: 0.001, reached: true}})
      BB.Test.MockSolver.set_result({:error, :unreachable, %{iterations: 50, residual: 0.5, reached: false}})
  """

  @behaviour BB.IK.Solver

  @doc """
  Set the result that solve/5 will return.
  """
  def set_result(result) do
    Process.put(:mock_solver_result, result)
  end

  @doc """
  Get the last call arguments (for assertions).
  """
  def last_call do
    Process.get(:mock_solver_last_call)
  end

  @impl BB.IK.Solver
  def solve(robot, state_or_positions, target_link, target, opts) do
    Process.put(:mock_solver_last_call, {robot, state_or_positions, target_link, target, opts})

    case Process.get(:mock_solver_result) do
      nil ->
        positions = %{}
        meta = %{iterations: 1, residual: 0.0, reached: true, reason: :converged}
        {:ok, positions, meta}

      result ->
        result
    end
  end
end
