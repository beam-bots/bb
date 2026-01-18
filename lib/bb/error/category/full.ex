# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Category.Full do
  @moduledoc """
  Category is at capacity for concurrent commands.

  Raised when attempting to execute a command in a category that has
  reached its `concurrency_limit`. Either wait for existing commands
  to complete, or cancel one to make room.
  """
  use BB.Error,
    class: :state,
    fields: [:category, :limit, :current]

  @type t :: %__MODULE__{
          category: atom(),
          limit: pos_integer(),
          current: non_neg_integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{category: category, limit: limit, current: current}) do
    "Category #{inspect(category)} is at capacity (#{current}/#{limit})"
  end
end
