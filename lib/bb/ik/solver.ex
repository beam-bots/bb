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
  - `Transform.t()` - 4x4 homogeneous transform (extracts position and quaternion)

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

  - `%BB.Error.Kinematics.UnknownLink{}` - Source or target link not found in robot
    topology; `:role` says which
  - `%BB.Error.Kinematics.NotAnAncestor{}` - Source link does not sit above the
    target link, carrying their nearest common ancestor
  - `%BB.Error.Kinematics.NoDofs{}` - Chain has no movable joints
  - `%BB.Error.Kinematics.Unreachable{}` - Target outside workspace
  - `%BB.Error.Kinematics.NoSolution{}` - Solver failed to converge

  ## Multi-DoF joints

  A chain may contain `:planar` and `:floating` joints, whose configurations are a
  `BB.Math.Transform2D` and a `BB.Math.Transform` rather than a float — see
  `BB.Robot.State`. Jacobian width is then the sum of degrees of freedom along the
  chain, and `BB.Robot.Kinematics.jacobian_columns/2` maps each column back to its
  joint and degree of freedom.

  Not every algorithm can handle this. A Jacobian-based solver is
  dimension-agnostic and needs no special case; a heuristic that repositions
  joints along a chain has no meaningful interpretation for a 6-DoF base. A solver
  that cannot should say so clearly rather than returning a wrong answer — and
  since `source_link` lets the caller scope the chain, a floating-base robot is
  still solvable by such a solver as long as the chain asked for excludes the
  floating joint.
  """

  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.NoSolution
  alias BB.Error.Kinematics.NotAnAncestor
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Error.Kinematics.Unreachable
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot

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
  - `Transform.t()` - 4x4 homogeneous transform (extracts position and quaternion)
  """
  @type target ::
          Vec3.t()
          | {Vec3.t(), orientation_target()}
          | Transform.t()

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
          UnknownLink.t()
          | NotAnAncestor.t()
          | NoDofs.t()
          | Unreachable.t()
          | NoSolution.t()

  @type solve_result ::
          {:ok, positions(), meta()}
          | {:error, kinematics_error()}

  @doc """
  Solve inverse kinematics for a target link to reach a target position/pose.

  ## Parameters

  - `robot` - The BB.Robot struct containing topology and joint information
  - `state_or_configurations` - Either a BB.Robot.State or a map of joint configurations
  - `source_link` - The link the chain starts at; must be an ancestor of `target_link`
  - `target_link` - The name of the link to position (end-effector)
  - `target` - Target position `{x, y, z}` or 4x4 pose transform
  - `opts` - Solver options

  ## Returns

  - `{:ok, configurations, meta}` - Successfully solved; configurations map and metadata
  - `{:error, error}` - Failed to solve; error struct contains all metadata

  Error structs include `:positions` with best-effort joint values when applicable.

  ## Why `source_link` has no default

  Defaulting it to the root is right for a fixed-base arm and precisely wrong for
  a robot whose base floats, where it silently drags a 6-DoF joint into a problem
  that has no business containing one. A default that is correct for one class of
  robot and quietly wrong for another is the footgun this parameter exists to
  remove, so every solve states its own scope:

      # The chain contains only revolute joints, so the floating base is simply
      # not part of the problem.
      solver.solve(robot, state, :body, :front_left_foot, target, opts)

      # The whole tree, said out loud.
      solver.solve(robot, state, BB.Robot.root_link(robot), :gripper, target, opts)

  Derive the chain with `BB.Robot.path_between/3`, which reports an unknown
  source, an unknown target, and a source that isn't above the target as three
  distinct errors.
  """
  @callback solve(
              robot :: Robot.t(),
              state_or_configurations :: Robot.State.t() | positions(),
              source_link :: atom(),
              target_link :: atom(),
              target :: target(),
              opts :: opts()
            ) :: solve_result()
end
