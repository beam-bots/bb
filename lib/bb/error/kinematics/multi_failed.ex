# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.MultiFailed do
  @moduledoc """
  Multi-target inverse kinematics failed.

  Raised when a multi-target IK operation fails for one or more targets.
  Contains the link that failed, the underlying error, and partial results
  from any successful targets.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:failed_link, :error, :partial_results]

  @type t :: %__MODULE__{
          failed_link: atom(),
          error: term(),
          partial_results: map()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{failed_link: link, error: error, partial_results: results}) do
    successful_count = map_size(results)

    "Multi-target IK failed for #{inspect(link)}: #{format_error(error)}. " <>
      "#{successful_count} target(s) solved before failure."
  end

  defp format_error(%{__struct__: _} = error), do: Exception.message(error)
  defp format_error(error), do: inspect(error)
end
