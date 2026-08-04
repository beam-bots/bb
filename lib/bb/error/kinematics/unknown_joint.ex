# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.UnknownJoint do
  @moduledoc """
  Joint not found in robot topology.

  Returned when a lookup names a joint the robot does not have.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:joint, :robot]

  @type t :: %__MODULE__{
          joint: atom(),
          robot: atom() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{joint: joint, robot: robot}) do
    robot_str = if robot, do: " in #{inspect(robot)}", else: ""
    "Unknown joint: #{inspect(joint)} not found#{robot_str}"
  end
end
