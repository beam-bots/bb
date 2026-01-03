# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.Server do
  @moduledoc """
  GenServer wrapper for command callback modules.

  This module manages the lifecycle of user-defined command modules, handling:
  - Parameter reference resolution at startup
  - Subscription to parameter changes
  - Safety state change notifications
  - Delegation of GenServer callbacks to user module
  - Result extraction and delivery to awaiting callers

  Command servers are temporary - they are not restarted on crash.
  """

  use GenServer

  alias BB.Command.Context
  alias BB.Command.ResultCache
  alias BB.Error.State.CommandCrashed
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.PubSub
  alias BB.Server.ParamResolution
  alias BB.StateMachine.Transition

  require Logger

  defstruct [
    :callback_module,
    :context,
    :goal,
    :execution_id,
    :runtime_pid,
    :timeout_ref,
    :resolved_opts,
    :raw_opts,
    :param_subscriptions,
    :awaiting,
    :user_state
  ]

  @type t :: %__MODULE__{
          callback_module: module(),
          context: Context.t(),
          goal: BB.Command.goal(),
          execution_id: reference(),
          runtime_pid: pid(),
          timeout_ref: reference() | nil,
          resolved_opts: keyword(),
          raw_opts: keyword(),
          param_subscriptions: %{[atom()] => atom()},
          awaiting: [GenServer.from()],
          user_state: term()
        }

  @doc false
  def start_link(init_arg) do
    GenServer.start_link(__MODULE__, init_arg)
  end

  @impl GenServer
  def init(init_arg) do
    callback_module = Keyword.fetch!(init_arg, :callback_module)
    context = Keyword.fetch!(init_arg, :context)
    goal = Keyword.fetch!(init_arg, :goal)
    execution_id = Keyword.fetch!(init_arg, :execution_id)
    runtime_pid = Keyword.fetch!(init_arg, :runtime_pid)
    raw_opts = Keyword.get(init_arg, :options, [])
    timeout = Keyword.get(init_arg, :timeout)

    # Subscribe to safety state changes
    PubSub.subscribe(context.robot_module, [:state_machine])

    # Resolve ParamRefs and subscribe to changes (use robot_state from context to avoid deadlock)
    {param_subscriptions, resolved_opts} =
      ParamResolution.resolve_and_subscribe(raw_opts, context.robot_state)

    # Start timeout timer if specified
    timeout_ref =
      if timeout && timeout != :infinity do
        Process.send_after(self(), :command_timeout, timeout)
      end

    # Build opts for user's init callback
    user_opts =
      Keyword.merge(resolved_opts,
        bb: %{robot: context.robot_module},
        goal: goal,
        context: context
      )

    case wrap_callback(nil, callback_module, :init, [user_opts]) do
      {:ok, user_state} ->
        state = %__MODULE__{
          callback_module: callback_module,
          context: context,
          goal: goal,
          execution_id: execution_id,
          runtime_pid: runtime_pid,
          timeout_ref: timeout_ref,
          resolved_opts: resolved_opts,
          raw_opts: raw_opts,
          param_subscriptions: param_subscriptions,
          awaiting: [],
          user_state: user_state
        }

        {:ok, state, {:continue, :execute}}

      {:stop, reason} ->
        {:stop, reason}
    end
  end

  @impl GenServer
  def handle_continue(:execute, state) do
    case wrap_callback(state, state.callback_module, :handle_command, [
           state.goal,
           state.context,
           state.user_state
         ]) do
      {:noreply, user_state} ->
        {:noreply, %{state | user_state: user_state}}

      {:noreply, user_state, action} ->
        {:noreply, %{state | user_state: user_state}, action}

      {:stop, reason, user_state} ->
        {:stop, reason, %{state | user_state: user_state}}
    end
  end

  def handle_continue(continue_arg, state) do
    case wrap_callback(state, state.callback_module, :handle_continue, [
           continue_arg,
           state.user_state
         ]) do
      {:noreply, user_state} ->
        {:noreply, %{state | user_state: user_state}}

      {:noreply, user_state, action} ->
        {:noreply, %{state | user_state: user_state}, action}

      {:stop, reason, user_state} ->
        {:stop, reason, %{state | user_state: user_state}}
    end
  end

  @impl GenServer
  def handle_call(:await, from, state) do
    # Don't reply now - will reply in terminate with result
    {:noreply, %{state | awaiting: [from | state.awaiting]}}
  end

  def handle_call(request, from, state) do
    case wrap_callback(state, state.callback_module, :handle_call, [
           request,
           from,
           state.user_state
         ]) do
      {:reply, reply, user_state} ->
        {:reply, reply, %{state | user_state: user_state}}

      {:reply, reply, user_state, action} ->
        {:reply, reply, %{state | user_state: user_state}, action}

      {:noreply, user_state} ->
        {:noreply, %{state | user_state: user_state}}

      {:noreply, user_state, action} ->
        {:noreply, %{state | user_state: user_state}, action}

      {:stop, reason, user_state} ->
        {:stop, reason, %{state | user_state: user_state}}

      {:stop, reason, reply, user_state} ->
        {:stop, reason, reply, %{state | user_state: user_state}}
    end
  end

  @impl GenServer
  def handle_cast(request, state) do
    case wrap_callback(state, state.callback_module, :handle_cast, [request, state.user_state]) do
      {:noreply, user_state} ->
        {:noreply, %{state | user_state: user_state}}

      {:noreply, user_state, action} ->
        {:noreply, %{state | user_state: user_state}, action}

      {:stop, reason, user_state} ->
        {:stop, reason, %{state | user_state: user_state}}
    end
  end

  @impl GenServer
  def handle_info({:bb, [:state_machine], %{payload: %Transition{to: new_safety_state}}}, state)
      when new_safety_state in [:disarming, :disarmed, :error] do
    case wrap_callback(state, state.callback_module, :handle_safety_state_change, [
           new_safety_state,
           state.user_state
         ]) do
      {:continue, user_state} ->
        {:noreply, %{state | user_state: user_state}}

      {:stop, reason, user_state} ->
        {:stop, reason, %{state | user_state: user_state}}
    end
  end

  def handle_info({:bb, [:param | param_path], %{payload: %ParameterChanged{}}}, state) do
    case ParamResolution.handle_change(
           param_path,
           state.param_subscriptions,
           state.raw_opts,
           state.context.robot_state
         ) do
      {:changed, new_resolved} ->
        case wrap_callback(state, state.callback_module, :handle_options, [
               new_resolved,
               state.user_state
             ]) do
          {:ok, new_user_state} ->
            {:noreply, %{state | resolved_opts: new_resolved, user_state: new_user_state}}

          {:stop, reason} ->
            {:stop, reason, state}
        end

      :ignored ->
        {:noreply, state}
    end
  end

  def handle_info(:command_timeout, state) do
    {:stop, :timeout, state}
  end

  def handle_info(msg, state) do
    case wrap_callback(state, state.callback_module, :handle_info, [msg, state.user_state]) do
      {:noreply, user_state} ->
        {:noreply, %{state | user_state: user_state}}

      {:noreply, user_state, action} ->
        {:noreply, %{state | user_state: user_state}, action}

      {:stop, reason, user_state} ->
        {:stop, reason, %{state | user_state: user_state}}
    end
  end

  @impl GenServer
  def terminate(reason, state) do
    # Cancel timeout timer
    if state.timeout_ref, do: Process.cancel_timer(state.timeout_ref)

    # Extract result from user state
    result =
      try do
        state.callback_module.result(state.user_state)
      rescue
        e ->
          Logger.error(
            "Command #{inspect(state.callback_module)} result/1 raised: #{Exception.message(e)}"
          )

          {:error, {:result_failed, e}}
      end

    # Store result in cache for callers who haven't awaited yet
    ResultCache.store(self(), result)

    # Reply to all awaiting callers
    for from <- state.awaiting do
      GenServer.reply(from, result)
    end

    # Report completion to Runtime
    GenServer.cast(state.runtime_pid, {:command_complete, state.execution_id, result})

    # Call user terminate
    try do
      state.callback_module.terminate(reason, state.user_state)
    rescue
      e ->
        Logger.error(
          "Command #{inspect(state.callback_module)} terminate/2 raised: #{Exception.message(e)}"
        )
    end

    :ok
  end

  # Wraps user callbacks in try/rescue to ensure awaiters are notified on crash
  defp wrap_callback(state, module, function, args) do
    apply(module, function, args)
  rescue
    exception ->
      stacktrace = __STACKTRACE__

      # Notify all awaiting processes of the crash (if state exists)
      if state do
        error =
          CommandCrashed.exception(
            command: state.callback_module,
            exception: exception
          )

        for from <- state.awaiting do
          GenServer.reply(from, {:error, error})
        end

        # Report failure to Runtime
        GenServer.cast(state.runtime_pid, {:command_crashed, state.execution_id, error})
      end

      # Re-raise to trigger normal GenServer crash handling
      reraise exception, stacktrace
  end
end
