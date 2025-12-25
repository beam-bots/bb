# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Diagnostic do
  @moduledoc """
  Diagnostic reporting for monitoring and awareness.

  Diagnostics are separate from control errors - they provide observability
  into system health without affecting control flow. Following the ROS2 model,
  diagnostics use four levels that indicate component health status.

  ## Diagnostic Levels

  - `:ok` - Component operating normally
  - `:warn` - Unusual condition, but operation continues
  - `:error` - Component has failed or is degraded
  - `:stale` - No recent updates from component (timeout/disconnect)

  ## Usage

  Publish diagnostics via telemetry:

      BB.Diagnostic.publish(
        component: [:robot, :arm, :elbow],
        level: :warn,
        message: "Motor temperature elevated",
        values: %{temperature: 65.2, threshold: 70.0}
      )

  Subscribe to diagnostics in your application:

      :telemetry.attach(
        "my-diagnostic-handler",
        [:bb, :diagnostic],
        &MyApp.handle_diagnostic/4,
        nil
      )

  ## Integration with bb_liveview

  The `bb_liveview` package provides a diagnostic dashboard that aggregates
  and displays diagnostics from all robot components. It subscribes to the
  `[:bb, :diagnostic]` telemetry event automatically.

  ## Separation from Control Errors

  Diagnostics are for **awareness** - they inform operators about system state
  but don't halt operations. Control errors (`BB.Error.*`) are for **control
  flow** - they're returned from functions and affect program execution.

  | Concern | Mechanism | Purpose |
  |---------|-----------|---------|
  | Diagnostics | Telemetry events | Monitoring dashboards |
  | Control errors | Return values | Program control flow |

  A component may publish a `:warn` diagnostic while continuing to operate,
  or publish an `:error` diagnostic when it has failed. The safety system
  (`BB.Safety`) handles critical failures independently.
  """

  @type level :: :ok | :warn | :error | :stale

  @type t :: %__MODULE__{
          component: [atom()],
          level: level(),
          message: String.t(),
          values: map(),
          timestamp: DateTime.t()
        }

  @enforce_keys [:component, :level, :message]
  defstruct [:component, :level, :message, values: %{}, timestamp: nil]

  @doc """
  Creates a new diagnostic struct.

  ## Options

  - `:component` (required) - Path to the component, e.g. `[:robot, :arm, :elbow]`
  - `:level` (required) - One of `:ok`, `:warn`, `:error`, `:stale`
  - `:message` (required) - Human-readable description
  - `:values` - Map of diagnostic values (default: `%{}`)
  - `:timestamp` - When the diagnostic was generated (default: `DateTime.utc_now/0`)

  ## Examples

      BB.Diagnostic.new(
        component: [:my_robot, :gripper],
        level: :ok,
        message: "Gripper operating normally"
      )

      BB.Diagnostic.new(
        component: [:my_robot, :arm, :shoulder],
        level: :warn,
        message: "Motor temperature elevated",
        values: %{temperature: 65.2, threshold: 70.0}
      )
  """
  @spec new(keyword()) :: t()
  def new(opts) do
    %__MODULE__{
      component: Keyword.fetch!(opts, :component),
      level: Keyword.fetch!(opts, :level),
      message: Keyword.fetch!(opts, :message),
      values: Keyword.get(opts, :values, %{}),
      timestamp: Keyword.get(opts, :timestamp) || DateTime.utc_now()
    }
  end

  @doc """
  Publishes a diagnostic event via telemetry.

  This is the primary way to emit diagnostics. The event is published to
  `[:bb, :diagnostic]` with the diagnostic struct as metadata.

  Accepts either a `BB.Diagnostic` struct or keyword options (which are
  passed to `new/1`).

  ## Examples

      # With keyword options
      BB.Diagnostic.publish(
        component: [:my_robot, :battery],
        level: :warn,
        message: "Battery low",
        values: %{percentage: 15, threshold: 20}
      )

      # With struct
      diagnostic = BB.Diagnostic.new(component: [:my_robot], level: :ok, message: "OK")
      BB.Diagnostic.publish(diagnostic)
  """
  @spec publish(t() | keyword()) :: :ok
  def publish(%__MODULE__{} = diagnostic) do
    :telemetry.execute(
      [:bb, :diagnostic],
      %{},
      diagnostic
    )
  end

  def publish(opts) when is_list(opts) do
    opts
    |> new()
    |> publish()
  end

  @doc """
  Convenience function to publish an `:ok` diagnostic.

  ## Examples

      BB.Diagnostic.ok([:my_robot, :arm], "Arm calibrated successfully")

      BB.Diagnostic.ok([:my_robot, :gripper], "Gripper ready",
        values: %{grip_force: 10.5}
      )
  """
  @spec ok([atom()], String.t(), keyword()) :: :ok
  def ok(component, message, opts \\ []) do
    opts
    |> Keyword.merge(component: component, level: :ok, message: message)
    |> publish()
  end

  @doc """
  Convenience function to publish a `:warn` diagnostic.

  ## Examples

      BB.Diagnostic.warn([:my_robot, :motor], "Temperature elevated",
        values: %{temperature: 65.0, threshold: 70.0}
      )
  """
  @spec warn([atom()], String.t(), keyword()) :: :ok
  def warn(component, message, opts \\ []) do
    opts
    |> Keyword.merge(component: component, level: :warn, message: message)
    |> publish()
  end

  @doc """
  Convenience function to publish an `:error` diagnostic.

  ## Examples

      BB.Diagnostic.error([:my_robot, :sensor], "Sensor disconnected",
        values: %{last_reading: ~U[2025-01-15 10:30:00Z]}
      )
  """
  @spec error([atom()], String.t(), keyword()) :: :ok
  def error(component, message, opts \\ []) do
    opts
    |> Keyword.merge(component: component, level: :error, message: message)
    |> publish()
  end

  @doc """
  Convenience function to publish a `:stale` diagnostic.

  Used when a component hasn't reported recently, indicating potential
  disconnect or hang.

  ## Examples

      BB.Diagnostic.stale([:my_robot, :camera], "No frames received",
        values: %{last_frame: ~U[2025-01-15 10:25:00Z], timeout_ms: 5000}
      )
  """
  @spec stale([atom()], String.t(), keyword()) :: :ok
  def stale(component, message, opts \\ []) do
    opts
    |> Keyword.merge(component: component, level: :stale, message: message)
    |> publish()
  end
end
