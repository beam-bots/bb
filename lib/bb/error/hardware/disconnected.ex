# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware.Disconnected do
  @moduledoc """
  Hardware device is disconnected or not responding.

  Raised when a device that was previously connected is no longer reachable.
  """
  use BB.Error, class: :hardware, fields: [:device, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{device: device, reason: nil}) do
    "Hardware disconnected: #{inspect(device)} is not responding"
  end

  def message(%{device: device, reason: reason}) do
    "Hardware disconnected: #{inspect(device)} - #{inspect(reason)}"
  end
end
