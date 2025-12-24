# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.Singularity do
  @moduledoc """
  Robot is near or at a kinematic singularity.

  Raised when the robot configuration is near a singular point where
  the Jacobian becomes ill-conditioned and motion control degrades.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:joint_positions, :manipulability, :threshold]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{joint_positions: _, manipulability: manip, threshold: threshold}) do
    "Kinematic singularity: manipulability #{manip} below threshold #{threshold}"
  end
end
