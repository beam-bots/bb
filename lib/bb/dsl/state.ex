# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.State do
  @moduledoc """
  A custom operational state for the robot.

  States define the operational context the robot can be in (beyond the
  built-in ones). Commands specify which states they can run in via
  `allowed_states`, and can transition the robot to new states via
  `next_state:` in their result.

  `built_in/0` lists the states every robot has without declaring them.
  """

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            name: nil,
            doc: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          name: atom,
          doc: String.t() | nil
        }

  @doc """
  The states every robot has without declaring them.

  `:idle` is the default operational state. `:disarmed` is owned by
  `BB.Safety` rather than the operational state machine, but it belongs to
  the same set: commands name it in `allowed_states` (that is how `arm` is
  reachable at all) and `BB.Command.Disarm` returns it as its `next_state`.

  `:disarming` and `:error` are deliberately absent. They are safety states
  a command can neither run in nor transition to — `BB.Robot.Runtime` refuses
  every command while the robot is in either — so admitting them would only
  let a command pretend the hardware was in a state it isn't.

  A robot that declares one of these itself keeps its own definition.
  """
  @spec built_in() :: [%__MODULE__{}]
  def built_in do
    [
      %__MODULE__{
        name: :idle,
        doc: "Default idle state - robot is armed and ready for commands"
      },
      %__MODULE__{
        name: :disarmed,
        doc: "The robot is not armed. Owned by `BB.Safety` rather than the robot's own states."
      }
    ]
  end

  @doc """
  The names of the states every robot has without declaring them.
  """
  @spec built_in_names() :: [atom]
  def built_in_names, do: Enum.map(built_in(), & &1.name)

  @doc """
  State names a robot may not declare.

  `BB.Safety` owns these, and `BB.Robot.Runtime.state/1` returns them ahead of
  whatever the operational state machine holds. A robot state sharing one of
  their names could therefore never be observed, and a command allowed in it
  could never run — `check_state_allowed/2` refuses every command while the
  robot is disarming or in error.

  `:disarmed` is not reserved: it is a built-in, and redeclaring it only
  changes its documentation. See `built_in/0`.
  """
  @spec reserved_names() :: [atom]
  def reserved_names, do: [:disarming, :error]
end
