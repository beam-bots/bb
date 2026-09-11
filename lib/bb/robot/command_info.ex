# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.CommandInfo do
  @moduledoc """
  Information about a currently executing command.

  The runtime keeps this for admission control and preemption. Introspection
  goes through `BB.Command.list/1`, which reads the robot's registry.
  """

  defstruct [:name, :pid, :ref, :category]

  @type t :: %__MODULE__{
          name: atom(),
          pid: pid(),
          ref: reference(),
          category: atom()
        }
end
