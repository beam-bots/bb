# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.State.JointHistoryDisabled do
  @moduledoc """
  Joint-state history was asked for, but the robot doesn't keep any.

  Returned by `BB.Robot.State.get_all_configurations_at/2` when the robot is
  configured with `joint_history false`, and for a state created without
  history, such as an IK solver's scratch table.
  """
  use BB.Error, class: :state, fields: []

  @type t :: %__MODULE__{}

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(_), do: "Joint-state history is disabled for this robot"
end
