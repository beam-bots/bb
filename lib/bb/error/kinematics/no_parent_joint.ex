# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.NoParentJoint do
  @moduledoc """
  A link has no parent joint because it is the root of the kinematic tree.

  Distinct from `BB.Error.Kinematics.UnknownLink` on purpose. The root link
  having no parent joint is a true structural fact, not a typo, so a caller
  walking up the tree gets a termination signal it can match on rather than
  being told its perfectly valid root link doesn't exist:

      defp ancestors(robot, link, acc) do
        case BB.Robot.parent_joint(robot, link) do
          {:ok, joint} -> ancestors(robot, joint.parent_link, [joint | acc])
          {:error, %NoParentJoint{}} -> {:ok, acc}
          {:error, reason} -> {:error, reason}
        end
      end
  """
  use BB.Error,
    class: :kinematics,
    fields: [:link]

  @type t :: %__MODULE__{link: atom()}

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{link: link}) do
    "#{inspect(link)} has no parent joint; it is the root of the kinematic tree"
  end
end
