# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety do
  @moduledoc """
  Safety system behaviour and API.

  Actuators, sensors, and controllers can implement the `BB.Safety` behaviour.
  The `disarm/1` callback is called by `BB.Safety.Controller` when:

  - The robot is disarmed via command
  - The robot supervisor crashes

  The callback receives the opts provided at registration and must be able to
  disable hardware without access to any GenServer state. This ensures safety
  even when the actuator process is dead.

  ## Safety States

  - `:disarmed` - Robot is safely disarmed, all disarm callbacks succeeded
  - `:armed` - Robot is armed and ready to operate
  - `:error` - Disarm attempted but one or more callbacks failed; hardware may not be safe

  When in `:error` state, the robot cannot be armed until `force_disarm/1` is called
  to acknowledge the error and reset to `:disarmed`.

  ## Example

      defmodule MyActuator do
        use GenServer
        @behaviour BB.Safety

        @impl BB.Safety
        def disarm(opts) do
          pin = Keyword.fetch!(opts, :pin)
          MyHardware.disable(pin)
          :ok
        end

        def init(opts) do
          BB.Safety.register(__MODULE__,
            robot: opts[:bb].robot,
            path: opts[:bb].path,
            opts: [pin: opts[:pin]]
          )
          # ...
        end
      end

  If your actuator doesn't need special disarm logic, you can implement a no-op:

      @impl BB.Safety
      def disarm(_opts), do: :ok

  ## Important Limitations

  The BEAM virtual machine provides soft real-time guarantees, not hard real-time.
  Disarm callbacks may be delayed by garbage collection, scheduler load, or other
  system activity. For safety-critical applications, always implement hardware-level
  safety controls as your primary protection.

  See the Safety documentation topic for detailed recommendations.
  """

  @doc """
  Make the hardware safe.

  Called with the opts provided at registration. Must work without GenServer state.
  """
  @callback disarm(opts :: keyword()) :: :ok | {:error, term()}

  # --- API (delegates to Controller) ---

  @doc """
  Check if a robot is armed.

  Fast ETS read - does not go through GenServer.
  """
  defdelegate armed?(robot_module), to: BB.Safety.Controller

  @doc """
  Get current safety state for a robot.

  Fast ETS read - does not go through GenServer.
  Returns `:armed`, `:disarmed`, or `:error`.
  """
  defdelegate state(robot_module), to: BB.Safety.Controller

  @doc """
  Check if a robot is in error state.

  Returns `true` if a disarm operation failed and the robot requires
  manual intervention via `force_disarm/1`.

  Fast ETS read - does not go through GenServer.
  """
  defdelegate in_error?(robot_module), to: BB.Safety.Controller

  @doc """
  Arm the robot.

  Goes through the safety controller GenServer to ensure proper state transitions.
  Cannot arm if robot is in `:error` state - must call `force_disarm/1` first.

  Returns `:ok` or `{:error, :already_armed | :in_error | :not_registered}`.
  """
  defdelegate arm(robot_module), to: BB.Safety.Controller

  @doc """
  Disarm the robot.

  Goes through the safety controller GenServer. Calls all registered `disarm/1`
  callbacks before updating state. If any callback fails, the robot transitions
  to `:error` state instead of `:disarmed`.

  Returns `:ok` or `{:error, :already_disarmed | {:disarm_failed, failures}}`.
  """
  defdelegate disarm(robot_module), to: BB.Safety.Controller

  @doc """
  Force disarm from error state.

  Use this function to acknowledge a failed disarm operation and reset the
  robot to `:disarmed` state. This should only be called after manually
  verifying that hardware is in a safe state.

  **WARNING**: This bypasses safety checks. Only use when you have manually
  verified that all actuators are disabled and the robot is safe.

  Returns `:ok` or `{:error, :not_in_error | :not_registered}`.
  """
  defdelegate force_disarm(robot_module), to: BB.Safety.Controller

  @doc """
  Register a safety handler (actuator/sensor/controller).

  Called by processes in their `init/1`. The opts should contain all
  hardware-specific parameters needed to call `disarm/1` without GenServer state.

  Writes directly to ETS to avoid blocking on the Controller's mailbox.

  ## Options

  - `:robot` (required) - The robot module
  - `:path` (required) - The path to this component (for logging)
  - `:opts` - Hardware-specific options passed to `disarm/1`

  ## Example

      BB.Safety.register(__MODULE__,
        robot: MyRobot,
        path: [:arm, :shoulder_joint, :servo],
        opts: [pin: 18]
      )
  """
  defdelegate register(module, opts), to: BB.Safety.Controller
end
