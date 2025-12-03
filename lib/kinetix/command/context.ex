# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Command.Context do
  @moduledoc """
  Context provided to command handlers during execution.

  Contains references to the robot module, static topology, dynamic state,
  and the unique execution identifier.
  """

  alias Kinetix.Robot.State, as: RobotState

  defstruct [:robot_module, :robot, :robot_state, :execution_id]

  @type t :: %__MODULE__{
          robot_module: module(),
          robot: Kinetix.Robot.t(),
          robot_state: RobotState.t(),
          execution_id: reference()
        }
end
