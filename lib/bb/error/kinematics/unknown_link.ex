# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.UnknownLink do
  @moduledoc """
  Target link not found in robot topology.

  Raised when attempting to solve inverse kinematics for a link
  that does not exist in the robot's kinematic structure.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:target_link, :robot]

  @type t :: %__MODULE__{
          target_link: atom(),
          robot: atom() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{target_link: link, robot: robot}) do
    robot_str = if robot, do: " in #{inspect(robot)}", else: ""
    "Unknown link: #{inspect(link)} not found#{robot_str}"
  end
end
