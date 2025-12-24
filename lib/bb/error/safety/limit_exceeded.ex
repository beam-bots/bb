# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Safety.LimitExceeded do
  @moduledoc """
  Physical limit exceeded on a joint or actuator.

  Raised when position, velocity, or torque limits are exceeded.
  This is a critical safety error that triggers automatic disarm.
  """
  use BB.Error,
    class: :safety,
    fields: [:component_path, :limit_type, :measured_value, :limit_value, :unit]

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  def message(%{
        component_path: path,
        limit_type: type,
        measured_value: measured,
        limit_value: limit,
        unit: unit
      }) do
    path_str = format_path(path)
    unit_str = if unit, do: " #{unit}", else: ""

    "Safety limit exceeded: #{path_str} #{type} limit violated " <>
      "(measured: #{measured}#{unit_str}, limit: #{limit}#{unit_str})"
  end

  defp format_path(path) when is_list(path), do: Enum.join(path, ".")
  defp format_path(path), do: inspect(path)
end
