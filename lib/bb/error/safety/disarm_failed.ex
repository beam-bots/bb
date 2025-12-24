# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Safety.DisarmFailed do
  @moduledoc """
  Disarm callback failed for a component.

  Raised when a safety disarm callback fails to complete successfully.
  This indicates the hardware may be in an unsafe state and requires
  manual intervention.
  """
  use BB.Error,
    class: :safety,
    fields: [:component_path, :reason, :failures]

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  def message(%{component_path: nil, reason: reason, failures: [_ | _] = failures}) do
    failure_count = Enum.count(failures)

    "Disarm failed: #{failure_count} component(s) failed to disarm - #{inspect(reason)}. " <>
      "Hardware may be in unsafe state."
  end

  def message(%{component_path: path, reason: reason, failures: _}) do
    path_str = format_path(path)

    "Disarm failed for #{path_str}: #{inspect(reason)}. Hardware may be in unsafe state."
  end

  defp format_path(path) when is_list(path), do: Enum.join(path, ".")
  defp format_path(path), do: inspect(path)
end
