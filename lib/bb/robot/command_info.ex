# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.CommandInfo do
  @moduledoc """
  Information about a currently executing command.

  Tracks metadata for commands running in the robot runtime.
  """

  defstruct [:name, :pid, :ref, :category, :started_at]

  @type t :: %__MODULE__{
          name: atom(),
          pid: pid(),
          ref: reference(),
          category: atom(),
          started_at: DateTime.t()
        }
end
