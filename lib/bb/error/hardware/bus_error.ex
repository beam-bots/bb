# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware.BusError do
  @moduledoc """
  Communication bus error (I2C, serial, etc.).

  Raised when there's a low-level bus communication failure.
  """
  use BB.Error, class: :hardware, fields: [:bus, :address, :operation, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{bus: bus, address: address, operation: operation, reason: reason}) do
    "Bus error on #{bus} address #{format_address(address)}: #{operation} failed - #{inspect(reason)}"
  end

  defp format_address(address) when is_integer(address) do
    "0x#{Integer.to_string(address, 16)}"
  end

  defp format_address(address), do: inspect(address)
end
