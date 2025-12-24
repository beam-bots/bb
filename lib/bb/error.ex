# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error do
  @moduledoc """
  Structured error handling for the Beam Bots ecosystem.

  This module wraps `Splode.Error` with compile-time enforcement of the
  `BB.Error.Severity` protocol. All error types in BB must implement this
  protocol to ensure consistent severity classification.

  ## Usage

  Define error types using `use BB.Error`:

      defmodule BB.Error.Hardware.Timeout do
        use BB.Error, class: :hardware, fields: [:device, :timeout_ms]

        defimpl BB.Error.Severity do
          def severity(_), do: :error
        end

        def message(%{device: device, timeout_ms: timeout_ms}) do
          "Hardware timeout on \#{inspect(device)} after \#{timeout_ms}ms"
        end
      end

  ## Error Classes

  The following error classes are defined:

  - `:hardware` - Communication failures with physical devices
  - `:safety` - Safety system violations (always `:critical` severity)
  - `:kinematics` - Motion planning failures
  - `:invalid` - Configuration and validation errors
  - `:state` - State machine violations
  - `:protocol` - Low-level protocol failures (Robotis, I2C, etc.)

  ## Severity Protocol

  Each error must implement `BB.Error.Severity`, which returns one of:

  - `:critical` - Immediate safety response required
  - `:error` - Operation failed, may retry or degrade
  - `:warning` - Unusual condition, operation continues
  """

  alias BB.Error.Severity

  @typedoc "An error struct that implements `BB.Error.Severity`"
  @type t :: struct()

  # Dialyzer infers wrong types for protocol calls before consolidation
  @dialyzer {:nowarn_function, severity: 1, critical?: 1}

  @doc """
  Use this macro to define error types. Wraps `Splode.Error` and enforces
  `BB.Error.Severity` protocol implementation at compile time.

  ## Options

  - `:class` - Required. The error class (`:hardware`, `:safety`, etc.)
  - `:fields` - Optional. List of fields for this error type.

  ## Example

      defmodule BB.Error.Safety.LimitExceeded do
        use BB.Error,
          class: :safety,
          fields: [:joint, :limit_type, :measured, :limit]

        defimpl BB.Error.Severity do
          def severity(_), do: :critical
        end

        def message(%{joint: joint, limit_type: type, measured: measured, limit: limit}) do
          "Joint \#{inspect(joint)} \#{type} limit exceeded: \#{measured} vs limit \#{limit}"
        end
      end
  """
  defmacro __using__(opts) do
    quote do
      use Splode.Error, unquote(opts)
      @after_verify {BB.Error, :__verify_severity_impl__}
    end
  end

  @doc false
  def __verify_severity_impl__(module) do
    error_struct = struct(module, [])

    unless Severity.impl_for(error_struct) do
      raise CompileError,
        description: """
        #{inspect(module)} must implement the BB.Error.Severity protocol.

        Add this to your error module:

            defimpl BB.Error.Severity do
              def severity(_), do: :error  # or :critical or :warning
            end
        """
    end
  end

  @doc """
  Returns the severity of an error.

  Delegates to `BB.Error.Severity.severity/1`.
  """
  @spec severity(t()) :: :critical | :error | :warning
  def severity(error) do
    Severity.severity(error)
  end

  @doc """
  Returns `true` if the error is critical (severity = `:critical`).
  """
  @spec critical?(t()) :: boolean()
  def critical?(error) do
    severity(error) == :critical
  end
end
