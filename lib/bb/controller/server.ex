# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.Server do
  @moduledoc """
  Wrapper GenServer for controller callback modules.

  This module manages the lifecycle of user-defined controller modules, handling:
  - Parameter reference resolution at startup
  - Subscription to parameter changes
  - Delegation of GenServer callbacks to user module
  - Optional safety registration (only if controller implements `disarm/1`)

  User modules implement the `BB.Controller` behaviour and define callbacks.
  This server wraps them, providing the actual GenServer implementation.
  """

  use GenServer

  alias BB.Dsl.ParamRef
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.PubSub
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  defstruct [
    :callback_module,
    :resolved_opts,
    :raw_opts,
    :param_subscriptions,
    :bb,
    :user_state
  ]

  @type t :: %__MODULE__{
          callback_module: module(),
          resolved_opts: keyword(),
          raw_opts: keyword(),
          param_subscriptions: %{[atom()] => atom()},
          bb: %{robot: module(), path: [atom()]},
          user_state: term()
        }

  @doc false
  def start_link(init_arg) do
    GenServer.start_link(__MODULE__, init_arg)
  end

  @doc false
  def start_link(init_arg, opts) do
    GenServer.start_link(__MODULE__, init_arg, opts)
  end

  @impl GenServer
  def init(init_arg) do
    callback_module = Keyword.fetch!(init_arg, :__callback_module__)
    raw_opts = Keyword.delete(init_arg, :__callback_module__)
    bb = Keyword.fetch!(raw_opts, :bb)

    {param_subscriptions, resolved_opts} = resolve_param_refs(raw_opts, bb.robot)

    for param_path <- Map.keys(param_subscriptions) do
      PubSub.subscribe(bb.robot, [:param | param_path])
    end

    case callback_module.init(resolved_opts) do
      {:ok, user_state} ->
        {:ok,
         %__MODULE__{
           callback_module: callback_module,
           resolved_opts: resolved_opts,
           raw_opts: raw_opts,
           param_subscriptions: param_subscriptions,
           bb: bb,
           user_state: user_state
         }}

      {:ok, user_state, timeout_or_continue} ->
        {:ok,
         %__MODULE__{
           callback_module: callback_module,
           resolved_opts: resolved_opts,
           raw_opts: raw_opts,
           param_subscriptions: param_subscriptions,
           bb: bb,
           user_state: user_state
         }, timeout_or_continue}

      {:stop, reason} ->
        {:stop, reason}

      :ignore ->
        :ignore
    end
  end

  @impl GenServer
  def handle_info({:bb, [:param | param_path], %{payload: %ParameterChanged{}}}, state) do
    if Map.has_key?(state.param_subscriptions, param_path) do
      {_subs, new_resolved} = resolve_param_refs(state.raw_opts, state.bb.robot)

      case state.callback_module.handle_options(new_resolved, state.user_state) do
        {:ok, new_user_state} ->
          {:noreply, %{state | resolved_opts: new_resolved, user_state: new_user_state}}

        {:stop, reason} ->
          {:stop, reason, state}
      end
    else
      {:noreply, state}
    end
  end

  def handle_info(msg, state) do
    case state.callback_module.handle_info(msg, state.user_state) do
      {:noreply, new_user_state} ->
        {:noreply, %{state | user_state: new_user_state}}

      {:noreply, new_user_state, timeout_or_continue} ->
        {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}

      {:stop, reason, new_user_state} ->
        {:stop, reason, %{state | user_state: new_user_state}}
    end
  end

  @impl GenServer
  def handle_call(request, from, state) do
    case state.callback_module.handle_call(request, from, state.user_state) do
      {:reply, reply, new_user_state} ->
        {:reply, reply, %{state | user_state: new_user_state}}

      {:reply, reply, new_user_state, timeout_or_continue} ->
        {:reply, reply, %{state | user_state: new_user_state}, timeout_or_continue}

      {:noreply, new_user_state} ->
        {:noreply, %{state | user_state: new_user_state}}

      {:noreply, new_user_state, timeout_or_continue} ->
        {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}

      {:stop, reason, new_user_state} ->
        {:stop, reason, %{state | user_state: new_user_state}}

      {:stop, reason, reply, new_user_state} ->
        {:stop, reason, reply, %{state | user_state: new_user_state}}
    end
  end

  @impl GenServer
  def handle_cast(request, state) do
    case state.callback_module.handle_cast(request, state.user_state) do
      {:noreply, new_user_state} ->
        {:noreply, %{state | user_state: new_user_state}}

      {:noreply, new_user_state, timeout_or_continue} ->
        {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}

      {:stop, reason, new_user_state} ->
        {:stop, reason, %{state | user_state: new_user_state}}
    end
  end

  @impl GenServer
  def handle_continue(continue_arg, state) do
    case state.callback_module.handle_continue(continue_arg, state.user_state) do
      {:noreply, new_user_state} ->
        {:noreply, %{state | user_state: new_user_state}}

      {:noreply, new_user_state, timeout_or_continue} ->
        {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}

      {:stop, reason, new_user_state} ->
        {:stop, reason, %{state | user_state: new_user_state}}
    end
  end

  @impl GenServer
  def terminate(reason, state) do
    state.callback_module.terminate(reason, state.user_state)
  end

  defp resolve_param_refs(opts, robot_module) do
    robot_state = Runtime.get_robot_state(robot_module)

    {subscriptions, resolved} =
      Enum.reduce(opts, {%{}, []}, fn {key, value}, {subs, resolved} ->
        case value do
          %ParamRef{path: path} ->
            {:ok, resolved_value} = RobotState.get_parameter(robot_state, path)
            {Map.put(subs, path, key), [{key, resolved_value} | resolved]}

          _ ->
            {subs, [{key, value} | resolved]}
        end
      end)

    {subscriptions, Enum.reverse(resolved)}
  end
end
