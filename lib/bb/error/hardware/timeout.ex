# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware.Timeout do
  @moduledoc """
  Communication timeout with a hardware device.

  Raised when a device doesn't respond within the expected time.
  """
  use BB.Error, class: :hardware, fields: [:device, :operation, :timeout_ms]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{device: device, operation: operation, timeout_ms: timeout_ms}) do
    "Hardware timeout: #{inspect(device)} did not respond to #{operation} within #{timeout_ms}ms"
  end
end
