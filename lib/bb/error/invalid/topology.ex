# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Topology do
  @moduledoc """
  Robot topology configuration error.

  Raised during DSL compilation when the robot topology is invalid
  (e.g., circular references, missing links, invalid joint types).
  """
  use BB.Error,
    class: :invalid,
    fields: [:module, :location, :message]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{module: module, location: location, message: msg}) do
    location_str = if location, do: " at #{format_location(location)}", else: ""
    "Invalid topology in #{inspect(module)}#{location_str}: #{msg}"
  end

  defp format_location(location) when is_list(location), do: Enum.join(location, ".")
  defp format_location(location), do: inspect(location)
end
