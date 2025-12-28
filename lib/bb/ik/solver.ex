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

  Solvers accept targets as:
  - `Vec3.t()` - Position only
  - `{Vec3.t(), orientation}` - Position with orientation constraint
  - `Nx.Tensor.t()` - 4x4 homogeneous transform (extracts position and quaternion)

  Orientation can be specified as:
  - `:none` - Position only (default)
  - `{:axis, Vec3.t()}` - Tool pointing direction (end-effector Z-axis alignment)
  - `{:quaternion, Quaternion.t()}` - Full 6-DOF orientation

  ## Options

  Common options that solvers should support:
  - `:max_iterations` - Maximum solver iterations (default: 50)
  - `:tolerance` - Position convergence tolerance in metres (default: 1.0e-4)
  - `:orientation_tolerance` - Angular convergence tolerance in radians (default: 0.01)
  - `:strict_orientation` - If true, error when orientation unsatisfiable; if false, best-effort (default: false)
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
  alias BB.Quaternion
  alias BB.Robot
  alias BB.Vec3

  @type positions :: %{atom() => float()}

  @typedoc """
  Orientation target for IK solving.

  - `:none` - Position only (default)
  - `{:axis, Vec3.t()}` - Tool pointing direction (end-effector Z-axis)
  - `{:quaternion, Quaternion.t()}` - Full 6-DOF orientation
  """
  @type orientation_target ::
          :none
          | {:axis, Vec3.t()}
          | {:quaternion, Quaternion.t()}

  @typedoc """
  Target for IK solving.

  - `Vec3.t()` - Position only
  - `{Vec3.t(), orientation_target()}` - Position with orientation constraint
  - `Nx.Tensor.t()` - 4x4 transform (extracts position and quaternion)
  """
  @type target ::
          Vec3.t()
          | {Vec3.t(), orientation_target()}
          | Nx.Tensor.t()

  @type opts :: [
          {:max_iterations, pos_integer()}
          | {:tolerance, float()}
          | {:orientation_tolerance, float()}
          | {:strict_orientation, boolean()}
          | {:respect_limits, boolean()}
          | {:initial_positions, positions() | nil}
        ]

  @type meta :: %{
          iterations: non_neg_integer(),
          residual: float(),
          orientation_residual: float() | nil,
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
