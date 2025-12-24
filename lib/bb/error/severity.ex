# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defprotocol BB.Error.Severity do
  @moduledoc """
  Protocol for determining error severity.

  All error types in the BB ecosystem must implement this protocol.
  Implementation is enforced at compile time via the `use BB.Error` macro.

  ## Severity Levels

  - `:critical` - Immediate safety response required. For `:safety` class errors,
    this triggers automatic disarm.
  - `:error` - Operation failed. May retry or degrade gracefully.
  - `:warning` - Unusual condition, operation continues.
  """

  @type t :: :critical | :error | :warning

  @doc """
  Returns the severity level for this error.
  """
  @spec severity(t) :: t
  def severity(error)
end
