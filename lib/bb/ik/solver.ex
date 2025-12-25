# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.Solver do
  @moduledoc """
  Behaviour for inverse kinematics solvers in the BB ecosystem.

  This behaviour defines a common interface for IK solvers, allowing
  different algorithms (FABRIK, Jacobian, analytical, etc.) to be
  used interchangeably.

  ## Implementing a Solver

      defmodule MyApp.IK.CustomSolver do
        @behaviour BB.IK.Solver

        @impl true
        def solve(robot, state_or_positions, target_link, target, opts) do
          # Your implementation here
          {:ok, positions, meta}
        end
      end

  ## Target Types

  Solvers should accept targets as either:
  - A position tuple `{x, y, z}` in metres
  - A 4x4 homogeneous transform (Nx tensor) for full pose

  ## Options

  Common options that solvers should support:
  - `:max_iterations` - Maximum solver iterations (default: 50)
  - `:tolerance` - Convergence tolerance in metres (default: 1.0e-4)
  - `:respect_limits` - Whether to clamp to joint limits (default: true)
  - `:initial_positions` - Starting joint positions (default: from state)

  ## Error Types

  Solvers return structured errors from `BB.Error.Kinematics`:

  - `%BB.Error.Kinematics.UnknownLink{}` - Target link not found in robot topology
  - `%BB.Error.Kinematics.NoDofs{}` - Chain has no movable joints
  - `%BB.Error.Kinematics.Unreachable{}` - Target outside workspace
  - `%BB.Error.Kinematics.NoSolution{}` - Solver failed to converge
  """

  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.NoSolution
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Error.Kinematics.Unreachable
  alias BB.Robot

  @type positions :: %{atom() => float()}

  @type target ::
          {float(), float(), float()}
          | Nx.Tensor.t()

  @type opts :: [
          {:max_iterations, pos_integer()}
          | {:tolerance, float()}
          | {:respect_limits, boolean()}
          | {:initial_positions, positions() | nil}
        ]

  @type meta :: %{
          iterations: non_neg_integer(),
          residual: float(),
          reached: boolean()
        }

  @type kinematics_error ::
          UnknownLink.t() | NoDofs.t() | Unreachable.t() | NoSolution.t()

  @type solve_result ::
          {:ok, positions(), meta()}
          | {:error, kinematics_error()}

  @doc """
  Solve inverse kinematics for a target link to reach a target position/pose.

  ## Parameters

  - `robot` - The BB.Robot struct containing topology and joint information
  - `state_or_positions` - Either a BB.Robot.State or a map of joint positions
  - `target_link` - The name of the link to position (end-effector)
  - `target` - Target position `{x, y, z}` or 4x4 pose transform
  - `opts` - Solver options

  ## Returns

  - `{:ok, positions, meta}` - Successfully solved; positions map and metadata
  - `{:error, error}` - Failed to solve; error struct contains all metadata

  Error structs include `:positions` with best-effort joint values when applicable.
  """
  @callback solve(
              robot :: Robot.t(),
              state_or_positions :: Robot.State.t() | positions(),
              target_link :: atom(),
              target :: target(),
              opts :: opts()
            ) :: solve_result()
end
