# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.Unreachable do
  @moduledoc """
  Target pose is outside the robot's workspace.

  Raised when the inverse kinematics solver determines that the
  target position cannot be reached by the robot.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:target_link, :target_pose, :reason, :iterations, :residual, :positions]

  @type t :: %__MODULE__{
          target_link: atom(),
          target_pose: term(),
          reason: String.t() | nil,
          iterations: non_neg_integer() | nil,
          residual: float() | nil,
          positions: map() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{target_link: link, target_pose: pose, reason: reason}) do
    reason_str = if reason, do: " - #{reason}", else: ""
    "Target unreachable: cannot reach #{inspect(pose)} for link #{inspect(link)}#{reason_str}"
  end
end
