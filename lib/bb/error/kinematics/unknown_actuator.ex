# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.UnknownActuator do
  @moduledoc """
  Actuator not found in robot topology.

  Returned when a lookup names an actuator the robot does not have.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:actuator, :robot]

  @type t :: %__MODULE__{
          actuator: atom(),
          robot: atom() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{actuator: actuator, robot: robot}) do
    robot_str = if robot, do: " in #{inspect(robot)}", else: ""
    "Unknown actuator: #{inspect(actuator)} not found#{robot_str}"
  end
end
