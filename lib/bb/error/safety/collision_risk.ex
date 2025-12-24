# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Safety.CollisionRisk do
  @moduledoc """
  Potential collision detected.

  Raised when the system detects a risk of collision between robot
  components, with the environment, or with people.
  """
  use BB.Error,
    class: :safety,
    fields: [:component_path, :collision_type, :details]

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  def message(%{component_path: path, collision_type: type, details: details}) do
    path_str = format_path(path)
    details_str = if details, do: " - #{details}", else: ""

    "Collision risk detected: #{path_str} #{type}#{details_str}"
  end

  defp format_path(path) when is_list(path), do: Enum.join(path, ".")
  defp format_path(path), do: inspect(path)
end
