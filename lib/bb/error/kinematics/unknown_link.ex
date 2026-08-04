# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.UnknownLink do
  @moduledoc """
  Link not found in robot topology.

  Returned when a lookup names a link the robot does not have.

  `:role` says which parameter was at fault, so a lookup taking both ends of a
  chain — `BB.Robot.path_between/3` — can report an unknown source as a source
  rather than misattributing it to the target. It is `nil` for lookups that take
  only one link.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:link, :role, :robot]

  @type role :: :source | :target

  @type t :: %__MODULE__{
          link: atom(),
          role: role() | nil,
          robot: atom() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{link: link, role: role, robot: robot}) do
    role_str = if role, do: "#{role} link", else: "link"
    robot_str = if robot, do: " in #{inspect(robot)}", else: ""
    "Unknown #{role_str}: #{inspect(link)} not found#{robot_str}"
  end
end
