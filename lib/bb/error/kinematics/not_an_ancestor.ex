# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.NotAnAncestor do
  @moduledoc """
  A source link does not sit above a target link in the kinematic tree.

  Returned by `BB.Robot.path_between/3`, which is restricted to chains that
  descend from the source to the target.

  `common_ancestor` is always populated, which turns the message from a
  complaint into an instruction: it names the link the caller should have
  passed as the source. There is no "disconnected" case to represent —
  `BB.Dsl.TopologyTransformer` rejects any topology without exactly one root
  link, and the DSL's nesting makes cycles structurally impossible, so any two
  links in a robot share at least the root as a common ancestor.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:source_link, :target_link, :common_ancestor]

  @type t :: %__MODULE__{
          source_link: atom(),
          target_link: atom(),
          common_ancestor: atom()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{source_link: source, target_link: target, common_ancestor: ancestor}) do
    "#{inspect(source)} is not an ancestor of #{inspect(target)}; " <>
      "their nearest common ancestor is #{inspect(ancestor)}"
  end
end
