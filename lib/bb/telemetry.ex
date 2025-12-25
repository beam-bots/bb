# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Telemetry do
  @moduledoc """
  Telemetry events emitted by the BB framework.

  BB uses `:telemetry` for both performance monitoring and diagnostics.
  This module documents all events and provides helper functions for
  instrumentation.

  ## Performance Events (Spans)

  Performance events use `:telemetry.span/3` which emits `:start`, `:stop`,
  and `:exception` events automatically.

  ### Motion Events

  * `[:bb, :motion, :solve]` - IK solver execution
    * Start measurements: `%{system_time: integer}`
    * Stop measurements: `%{duration: native_time, monotonic_time: integer}`
    * Metadata: `%{robot: atom, target_link: atom, solver: module}`
    * Stop metadata adds: `%{iterations: integer, residual: float, reached: boolean}`

  * `[:bb, :motion, :move_to]` - Full move operation (solve + send)
    * Start measurements: `%{system_time: integer}`
    * Stop measurements: `%{duration: native_time}`
    * Metadata: `%{robot: atom, target_link: atom}`

  * `[:bb, :motion, :send_positions]` - Sending positions to actuators
    * Start measurements: `%{system_time: integer}`
    * Stop measurements: `%{duration: native_time}`
    * Metadata: `%{robot: atom, joint_count: integer, delivery: atom}`

  ### Kinematics Events

  * `[:bb, :kinematics, :forward]` - Forward kinematics computation
    * Start measurements: `%{system_time: integer}`
    * Stop measurements: `%{duration: native_time}`
    * Metadata: `%{robot: atom, target_link: atom}`

  ### Command Events

  * `[:bb, :command, :execute]` - Command execution
    * Start measurements: `%{system_time: integer}`
    * Stop measurements: `%{duration: native_time}`
    * Metadata: `%{robot: atom, command: atom, execution_id: reference}`

  ## Diagnostic Events

  * `[:bb, :diagnostic]` - Component health diagnostics
    * Measurements: `%{}`
    * Metadata: `%BB.Diagnostic{}` struct

  See `BB.Diagnostic` for details on diagnostic events.

  ## Subscribing to Events

  Use `:telemetry.attach/4` or `:telemetry.attach_many/4`:

      :telemetry.attach_many(
        "my-perf-handler",
        [
          [:bb, :motion, :solve, :stop],
          [:bb, :motion, :move_to, :stop]
        ],
        &MyApp.handle_perf_event/4,
        nil
      )

  ## Converting Duration

  Durations are in native time units. Convert to milliseconds:

      duration_ms = System.convert_time_unit(duration, :native, :millisecond)

  Or to microseconds for high-precision timing:

      duration_us = System.convert_time_unit(duration, :native, :microsecond)
  """

  @doc """
  Wraps a function in a telemetry span.

  This is a convenience wrapper around `:telemetry.span/3` that handles
  the common pattern of extracting metadata from the result.

  ## Parameters

  - `event` - The event name prefix (e.g., `[:bb, :motion, :solve]`)
  - `metadata` - Initial metadata map
  - `fun` - Function to execute, should return `{result, extra_metadata}`

  ## Examples

      BB.Telemetry.span([:bb, :motion, :solve], %{robot: :my_robot}, fn ->
        result = do_solve()
        {result, %{iterations: 10, residual: 0.001}}
      end)
  """
  @spec span([atom()], map(), (-> {any(), map()})) :: any()
  def span(event, metadata, fun) when is_list(event) and is_map(metadata) do
    :telemetry.span(event, metadata, fun)
  end

  @doc """
  Emits a telemetry event.

  Convenience wrapper around `:telemetry.execute/3`.

  ## Examples

      BB.Telemetry.emit([:bb, :custom, :event], %{count: 1}, %{robot: :my_robot})
  """
  @spec emit([atom()], map(), map()) :: :ok
  def emit(event, measurements, metadata) do
    :telemetry.execute(event, measurements, metadata)
  end
end
