# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.NoSolution do
  @moduledoc """
  Inverse kinematics solver failed to converge.

  Raised when the IK solver cannot find a solution within the
  maximum number of iterations.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:target_link, :target_pose, :iterations, :residual]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{target_link: link, iterations: iters, residual: residual}) do
    "IK solver failed: no solution found for #{inspect(link)} " <>
      "after #{iters} iterations (residual: #{Float.round(residual, 6)})"
  end
end
