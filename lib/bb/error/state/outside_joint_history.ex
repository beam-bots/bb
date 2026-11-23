# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.OutsideJointHistory do
  @moduledoc """
  A joint configuration was asked for at a time the history can't answer.

  Returned by `BB.Robot.State.get_all_configurations_at/2` in two cases,
  distinguished by `direction`:

  - `:past` - the time is older than `joint`'s oldest retained sample, either
    because it has aged out of the history or because it predates the robot
    starting. `boundary_time` is that oldest sample's time.
  - `:future` - the time is later than now. Nothing can be known about it yet,
    and the history refuses to extrapolate. `joint` is `nil`, since it applies
    to every joint, and `boundary_time` is the current monotonic time.

  All times are `System.monotonic_time(:nanosecond)` values.
  """
  use BB.Error,
    class: :state,
    fields: [:direction, :joint, :requested_time, :boundary_time]

  @type t :: %__MODULE__{
          direction: :past | :future,
          joint: atom() | nil,
          requested_time: integer(),
          boundary_time: integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{direction: :past, joint: joint} = error) do
    "No history for joint #{inspect(joint)} at #{error.requested_time}: " <>
      "its oldest retained sample is at #{error.boundary_time}"
  end

  def message(%{direction: :future} = error) do
    "Cannot look up joint configurations at #{error.requested_time}: " <>
      "it is later than now (#{error.boundary_time})"
  end
end
