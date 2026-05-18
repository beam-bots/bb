# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety do
  @moduledoc """
  Safety system API.

  This module provides the API for arming/disarming robots and managing safety state.
  The `disarm/1` callback that components implement is now defined in `BB.Controller`
  and `BB.Actuator` behaviours.

  ## Safety States

  - `:disarmed` - Robot is safely disarmed, all disarm callbacks succeeded
  - `:armed` - Robot is armed and ready to operate
  - `:disarming` - Disarm in progress, callbacks running concurrently
  - `:error` - Disarm attempted but one or more callbacks failed; hardware may not be safe

  When in `:error` state, the robot cannot be armed until `force_disarm/1` is called
  to acknowledge the error and reset to `:disarmed`.

  Disarm callbacks run concurrently with a timeout. If any callback fails or times out,
  the robot transitions to `:error` state.

  ## Implementing Disarm Callbacks

  Controllers and actuators implement the `disarm/1` callback via their behaviours:

      defmodule MyActuator do
        use GenServer
        use BB.Actuator

        @impl BB.Actuator
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

      @impl BB.Actuator
      def disarm(_opts), do: :ok

  ## Important Limitations

  The BEAM virtual machine provides soft real-time guarantees, not hard real-time.
  Disarm callbacks may be delayed by garbage collection, scheduler load, or other
  system activity. For safety-critical applications, always implement hardware-level
  safety controls as your primary protection.

  See the Safety documentation topic for detailed recommendations.
  """

  alias BB.Dsl.Command, as: DslCommand
  alias BB.Dsl.Info
  alias BB.Robot.Runtime
  alias BB.Safety.Controller

  # --- API (delegates to Controller) ---

  @doc """
  Check if a robot is armed.

  Fast ETS read - does not go through GenServer.
  """
  defdelegate armed?(robot_module), to: BB.Safety.Controller

  @doc """
  Get current safety state for a robot.

  Fast ETS read - does not go through GenServer.
  Returns `:armed`, `:disarmed`, `:disarming`, or `:error`.
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
  Check if a robot is currently disarming.

  Returns `true` while disarm callbacks are running.

  Fast ETS read - does not go through GenServer.
  """
  defdelegate disarming?(robot_module), to: BB.Safety.Controller

  @doc """
  Arm the robot.

  If the robot's DSL declares a command with `arm true` (set explicitly, or
  implicitly when the handler is `BB.Command.Arm`), this function dispatches
  that command via `BB.Robot.Runtime.execute/3` and awaits its result. The
  command is responsible for performing whatever work the user wants done
  on arming (e.g. moving joints to a home pose) and for flipping safety state
  via `BB.Safety.Controller.arm/1`.

  If no `arm`-flagged command is defined, this function falls through to the
  safety controller's direct state-flip behaviour — the historical default.

  Cannot arm if the robot is in `:error` state; call `force_disarm/1` first.

  Returns `:ok` or `{:error, :already_armed | :in_error | :not_registered |
  term()}`. When routed through a command, the error reason is whatever the
  command returned (typically a `BB.Error.State` exception or a
  `:command_failed` tuple).
  """
  @spec arm(module()) :: :ok | {:error, term()}
  def arm(robot_module) do
    case routed_command(robot_module, :__bb_arm_command__) do
      nil -> Controller.arm(robot_module)
      command_name -> route_arm(robot_module, command_name)
    end
  end

  @doc """
  Disarm the robot.

  If the robot's DSL declares a command with `disarm true` (set explicitly, or
  implicitly when the handler is `BB.Command.Disarm`), this function
  dispatches that command via `BB.Robot.Runtime.execute/3` and awaits its
  result. The command is responsible for any pre-disarm work and for flipping
  safety state via `BB.Safety.Controller.disarm/2`. If the command returns
  successfully, the robot is in whatever state the command left it in
  (typically `:disarmed`). If the command returns an error before the safety
  state has been flipped, the robot is escalated to `:error` — by the issue's
  failure semantics, an incomplete disarm sequence means hardware may not be
  in a safe state.

  If no `disarm`-flagged command is defined, this function falls through to
  the safety controller's direct disarm behaviour — the historical default.

  ## Options

    * `:timeout` - timeout in milliseconds for each disarm callback. Defaults
      to 5000ms. Only applicable when no `disarm`-flagged command is defined;
      otherwise the command's own `:timeout` is used.

  Returns `:ok` or `{:error, :already_disarmed | {:disarm_failed, failures} |
  {:disarm_command_failed, reason} | term()}`.
  """
  @spec disarm(module(), keyword()) :: :ok | {:error, term()}
  def disarm(robot_module, opts \\ []) do
    case routed_command(robot_module, :__bb_disarm_command__) do
      nil -> Controller.disarm(robot_module, opts)
      command_name -> route_disarm(robot_module, command_name)
    end
  end

  defp routed_command(robot_module, fun) do
    if function_exported?(robot_module, fun, 0) do
      apply(robot_module, fun, [])
    end
  end

  defp route_arm(robot_module, command_name) do
    case Runtime.execute(robot_module, command_name, %{}) do
      {:ok, pid} ->
        case BB.Command.await(pid, await_timeout(robot_module, command_name)) do
          {:ok, _} -> :ok
          {:ok, _, _opts} -> :ok
          {:error, reason} -> {:error, reason}
        end

      {:error, _} = error ->
        error
    end
  end

  defp route_disarm(robot_module, command_name) do
    case Runtime.execute(robot_module, command_name, %{}) do
      {:ok, pid} ->
        result = BB.Command.await(pid, await_timeout(robot_module, command_name))
        handle_disarm_result(robot_module, result)

      {:error, _} = error ->
        error
    end
  end

  defp handle_disarm_result(_robot_module, {:ok, _}), do: :ok
  defp handle_disarm_result(_robot_module, {:ok, _, _opts}), do: :ok

  defp handle_disarm_result(robot_module, {:error, reason}) do
    case Controller.state(robot_module) do
      state when state in [:armed, :disarming] ->
        # Disarm command failed before safety state reached a terminal value —
        # park the robot in :error so the operator has to acknowledge it.
        :ok = Controller.transition_to_error(robot_module)
        {:error, {:disarm_command_failed, reason}}

      _ ->
        {:error, reason}
    end
  end

  # The await timeout must accommodate the command's own timeout (if set),
  # plus some headroom for the command server's terminate sequence. If the
  # command sets `:infinity`, we use `:infinity` here too.
  defp await_timeout(robot_module, command_name) do
    case command_timeout(robot_module, command_name) do
      :infinity -> :infinity
      ms when is_integer(ms) -> ms + 5_000
      _ -> :infinity
    end
  end

  defp command_timeout(robot_module, command_name) do
    robot_module
    |> Info.commands()
    |> Enum.find_value(:infinity, fn
      %DslCommand{name: ^command_name, timeout: timeout} -> timeout
      _ -> false
    end)
  end

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

  @doc """
  Report a hardware error from a component.

  Publishes a `BB.Safety.HardwareError` message to `[:safety, :error]` for
  subscribers to handle. This is a pure notification - it does not disarm the
  robot or change safety state.

  Components that detect an unrecoverable hardware fault should `raise` or
  exit instead of (or in addition to) calling this function. The supervisor
  will restart the offending process; if the restart budget on the topology
  supervisor is exhausted, the safety controller will force-disarm the robot.
  This is the OTP-native way to signal hardware failure: let it crash, and
  let supervision escalate.

  ## Parameters

  - `robot_module` - The robot module
  - `path` - Path to the component reporting the error (e.g., `[:dynamixel, :servo_1]`)
  - `error` - Component-specific error details

  ## Example

      # In a controller detecting servo overheating - publish for observers,
      # then crash so the supervisor decides whether to escalate:
      BB.Safety.report_error(MyRobot, [:dynamixel, :servo_1], {:hardware_error, 0x04})
      raise BB.Error.Hardware.Overheat, servo: 1
  """
  defdelegate report_error(robot_module, path, error), to: BB.Safety.Controller
end
