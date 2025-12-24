# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.SelfCollision do
  @moduledoc """
  Motion would cause self-collision.

  Raised when the planned motion trajectory would result in
  collision between robot links.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:link_a, :link_b, :joint_positions]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{link_a: a, link_b: b, joint_positions: _}) do
    "Self-collision detected: #{inspect(a)} would collide with #{inspect(b)}"
  end
end
