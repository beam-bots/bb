# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Safety.EmergencyStop do
  @moduledoc """
  Emergency stop triggered.

  Raised when an emergency stop condition is detected, either from
  hardware (e-stop button) or software safety systems.
  """
  use BB.Error,
    class: :safety,
    fields: [:source, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  def message(%{source: source, reason: nil}) do
    "Emergency stop triggered by #{inspect(source)}"
  end

  def message(%{source: source, reason: reason}) do
    "Emergency stop triggered by #{inspect(source)}: #{reason}"
  end
end
