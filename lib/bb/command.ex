# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command do
  @moduledoc """
  Behaviour for implementing robot commands.

  Commands are short-lived GenServers that can react to safety state changes
  and other messages during execution. The `handle_command/3` callback is the
  entry point, returning GenServer-style tuples.

  ## Example

      defmodule NavigateToPose do
        use BB.Command

        @impl BB.Command
        def handle_command(%{target_pose: pose}, context, state) do
          # Subscribe to position updates
          BB.PubSub.subscribe(context.robot_module, [:sensor, :position])

          # Start navigation
          send_navigation_command(pose)

          {:noreply, %{state | target: pose}}
        end

        @impl BB.Command
        def handle_info({:bb, [:sensor, :position], msg}, state) do
          if close_enough?(msg.payload.position, state.target) do
            {:stop, :normal, %{state | final_pose: msg.payload.position}}
          else
            {:noreply, state}
          end
        end

        @impl BB.Command
        def result(state) do
          {:ok, %{final_pose: state.final_pose}}
        end
      end

  ## State Transitions

  By default, when a command completes successfully, the robot transitions to
  `:idle`. Commands can override this by returning a `next_state` option from
  `result/1`:

      def result(state) do
        {:ok, :armed, next_state: :idle}
      end

  This is useful for commands like `Arm` and `Disarm` that need to control
  the robot's state machine.

  ## Execution Model

  Commands run as supervised GenServers spawned by the Runtime. The caller
  receives the command's pid and can use `BB.Command.await/2` or
  `BB.Command.yield/2` to get the result.

  ## Safety Handling

  Commands automatically subscribe to safety state changes. When the robot
  begins disarming, `handle_safety_state_change/2` is called. The default
  implementation stops the command with `:disarmed` reason. Override this
  callback to implement graceful shutdown or to continue execution during
  safety transitions.

  ## Parameterised Options

  Commands can receive options via child_spec format in the DSL:

      commands do
        command :move_joint do
          handler {MyMoveJointCommand, max_velocity: param([:motion, :max_velocity])}
        end
      end

  ParamRefs are resolved before `init/1` is called. When parameters change,
  `handle_options/2` is called with the new resolved options.
  """

  alias BB.Command.Context
  alias BB.Command.ResultCache
  alias BB.Robot.Runtime

  @type goal :: map()
  @type result :: term()
  @type state :: term()
  @type options :: [next_state: BB.Robot.Runtime.robot_state()]

  @doc """
  Initialise the command state.

  Called when the command server starts. Receives resolved options including:
  - `:bb` - Map with `:robot` (robot module)
  - `:goal` - The command goal (arguments)
  - `:context` - The command context

  The default implementation returns `{:ok, Map.new(opts)}`.
  """
  @callback init(opts :: keyword()) :: {:ok, state()} | {:stop, term()}

  @doc """
  Execute the command with the given goal.

  Called via `handle_continue(:execute)` immediately after `init/1`. This is
  the main entry point for command execution.

  The handler can:
  - Return `{:noreply, state}` to continue running (waiting for messages)
  - Return `{:stop, reason, state}` to complete immediately

  For commands that complete immediately, simply return `{:stop, :normal, state}`
  with the result stored in state.
  """
  @callback handle_command(goal(), Context.t(), state()) ::
              {:noreply, state()}
              | {:noreply, state(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, term(), state()}

  @doc """
  Extract the result when the command completes.

  Called in `terminate/2` to get the result to return to awaiting callers.

  ## Return Values

  - `{:ok, result}` - Command succeeded, robot transitions to `:idle`
  - `{:ok, result, options}` - Command succeeded with options:
    - `next_state: state` - Robot transitions to specified state instead of `:idle`
  - `{:error, reason}` - Command failed, robot transitions to `:idle`
  """
  @callback result(state()) ::
              {:ok, result()}
              | {:ok, result(), options()}
              | {:error, term()}

  @doc """
  Handle safety state changes.

  Called when the robot's safety state transitions to `:disarming`, `:disarmed`,
  or `:error`. The default implementation stops the command with `:disarmed`
  reason.

  Return `{:continue, state}` to keep the command running during safety
  transitions (use with care).
  """
  @callback handle_safety_state_change(
              new_state :: :disarming | :disarmed | :error,
              state()
            ) ::
              {:continue, state()}
              | {:stop, term(), state()}

  @doc """
  Handle parameter changes.

  Called when a parameter that this command depends on changes. The new
  resolved options are passed in. The default implementation returns
  `{:ok, state}` unchanged.
  """
  @callback handle_options(new_opts :: keyword(), state()) ::
              {:ok, state()}
              | {:stop, term()}

  @doc """
  Handle synchronous calls.

  Standard GenServer callback. The default implementation returns
  `{:reply, {:error, :not_implemented}, state}`.
  """
  @callback handle_call(request :: term(), from :: GenServer.from(), state()) ::
              {:reply, term(), state()}
              | {:reply, term(), state(), timeout() | :hibernate | {:continue, term()}}
              | {:noreply, state()}
              | {:noreply, state(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, term(), state()}
              | {:stop, term(), term(), state()}

  @doc """
  Handle asynchronous casts.

  Standard GenServer callback. The default implementation returns
  `{:noreply, state}`.
  """
  @callback handle_cast(request :: term(), state()) ::
              {:noreply, state()}
              | {:noreply, state(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, term(), state()}

  @doc """
  Handle other messages.

  Standard GenServer callback. The default implementation returns
  `{:noreply, state}`.
  """
  @callback handle_info(msg :: term(), state()) ::
              {:noreply, state()}
              | {:noreply, state(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, term(), state()}

  @doc """
  Handle continue instructions.

  Standard GenServer callback. The default implementation returns
  `{:noreply, state}`.
  """
  @callback handle_continue(continue :: term(), state()) ::
              {:noreply, state()}
              | {:noreply, state(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, term(), state()}

  @doc """
  Clean up when the command terminates.

  Standard GenServer callback. Called after the result has been extracted
  and sent to awaiting callers.
  """
  @callback terminate(reason :: term(), state()) :: term()

  @doc """
  Define the options schema for this command.

  Optional. If defined, options passed to the command handler will be
  validated against this schema.
  """
  @callback options_schema() :: Spark.Options.schema()

  @optional_callbacks [options_schema: 0]

  @doc """
  Await the command result, blocking until completion or timeout.

  Uses `GenServer.call` internally, so standard timeout semantics apply.
  If the command crashes, returns `{:error, {:command_failed, reason}}`.

  ## Examples

      {:ok, cmd} = MyRobot.navigate(target: pose)
      {:ok, result} = BB.Command.await(cmd)

      # With custom timeout
      {:ok, result} = BB.Command.await(cmd, 30_000)
  """
  @spec await(pid(), timeout()) :: {:ok, term()} | {:ok, term(), options()} | {:error, term()}
  def await(pid, timeout \\ 5000) do
    GenServer.call(pid, :await, timeout)
  catch
    :exit, {:noproc, _} ->
      # Process already terminated - check the result cache
      case ResultCache.fetch_and_delete(pid) do
        {:ok, result} -> result
        :error -> {:error, {:command_failed, :noproc}}
      end

    :exit, {:timeout, _} ->
      {:error, {:command_failed, :timeout}}

    :exit, {reason, _} ->
      # Process terminated during call - check the result cache
      case ResultCache.fetch_and_delete(pid) do
        {:ok, result} -> result
        :error -> {:error, {:command_failed, reason}}
      end
  end

  @doc """
  Non-blocking check for command completion.

  Returns `nil` if the command is still running (timeout), otherwise returns
  the result. Use this for polling-style waiting.

  ## Examples

      {:ok, cmd} = MyRobot.navigate(target: pose)

      case BB.Command.yield(cmd, 100) do
        nil -> IO.puts("Still running...")
        {:ok, result} -> IO.puts("Done!")
        {:error, reason} -> IO.puts("Failed: \#{inspect(reason)}")
      end
  """
  @spec yield(pid(), timeout()) ::
          {:ok, term()} | {:ok, term(), options()} | {:error, term()} | nil
  def yield(pid, timeout \\ 0) do
    GenServer.call(pid, :await, timeout)
  catch
    :exit, {:timeout, _} ->
      nil

    :exit, {:noproc, _} ->
      # Process already terminated - check the result cache
      case ResultCache.fetch_and_delete(pid) do
        {:ok, result} -> result
        :error -> {:error, {:command_failed, :noproc}}
      end

    :exit, {reason, _} ->
      # Process terminated during call - check the result cache
      case ResultCache.fetch_and_delete(pid) do
        {:ok, result} -> result
        :error -> {:error, {:command_failed, reason}}
      end
  end

  @doc """
  Cancel a running command.

  Stops the command server with `:cancelled` reason. Awaiting callers will
  receive `{:error, :cancelled}` (depending on how `result/1` handles this).
  """
  @spec cancel(pid()) :: :ok
  def cancel(pid) do
    GenServer.stop(pid, :cancelled)
  catch
    :exit, {:noproc, _} -> :ok
  end

  @doc """
  Transition to a new operational state during command execution.

  This function allows a command to change the robot's operational state
  mid-execution. This is useful for multi-phase commands where different
  phases require different contexts.

  ## Arguments

  - `context` - The command context (passed to `handle_command/3`)
  - `target_state` - The state to transition to (must be defined in DSL)

  ## Returns

  - `:ok` - Transition successful
  - `{:error, reason}` - Transition failed

  ## Example

      def handle_command(_goal, context, state) do
        # Start in processing state
        :ok = BB.Command.transition_state(context, :processing)
        # Do work...
        send(self(), :start_phase_two)
        {:noreply, state}
      end

      def handle_info(:start_phase_two, context, state) do
        # Move to finalising state
        :ok = BB.Command.transition_state(context, :finalising)
        # Do more work...
        {:stop, :normal, state}
      end

  """
  @spec transition_state(Context.t(), atom()) :: :ok | {:error, term()}
  def transition_state(%Context{} = context, target_state) when is_atom(target_state) do
    Runtime.transition_operational_state(
      context.robot_module,
      context.execution_id,
      target_state
    )
  end

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      @behaviour BB.Command

      @impl BB.Command
      def init(opts) do
        state =
          opts
          |> Map.new()
          |> Map.put_new(:result, nil)
          |> Map.put_new(:next_state, nil)

        {:ok, state}
      end

      @impl BB.Command
      def handle_safety_state_change(_new_state, state) do
        {:stop, :disarmed, state}
      end

      @impl BB.Command
      def handle_options(_new_opts, state), do: {:ok, state}

      @impl BB.Command
      def handle_call(_request, _from, state) do
        {:reply, {:error, :not_implemented}, state}
      end

      @impl BB.Command
      def handle_cast(_request, state), do: {:noreply, state}

      @impl BB.Command
      def handle_info(_msg, state), do: {:noreply, state}

      @impl BB.Command
      def handle_continue(_continue, state), do: {:noreply, state}

      @impl BB.Command
      def terminate(_reason, _state), do: :ok

      defoverridable init: 1,
                     handle_safety_state_change: 2,
                     handle_options: 2,
                     handle_call: 3,
                     handle_cast: 2,
                     handle_info: 2,
                     handle_continue: 2,
                     terminate: 2

      unquote(
        if schema_opts do
          quote do
            @__bb_options_schema unquote(schema_opts)
            @impl BB.Command
            def options_schema, do: @__bb_options_schema
            defoverridable options_schema: 0
          end
        end
      )
    end
  end
end
