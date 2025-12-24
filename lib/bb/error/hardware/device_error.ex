# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware.DeviceError do
  @moduledoc """
  Error reported by the hardware device itself.

  Raised when a device reports an error condition through its protocol.
  """
  use BB.Error, class: :hardware, fields: [:device, :error_code, :description]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{device: device, error_code: code, description: desc}) when not is_nil(desc) do
    "Device error from #{inspect(device)}: #{desc} (code: #{inspect(code)})"
  end

  def message(%{device: device, error_code: code, description: nil}) do
    "Device error from #{inspect(device)}: code #{inspect(code)}"
  end
end
