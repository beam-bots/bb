# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.Server do
  @moduledoc """
  Wrapper GenServer for estimator callback modules.

  Responsibilities:

  - Resolves parameter references in opts at startup and on parameter
    changes (mirroring `BB.Sensor.Server` / `BB.Controller.Server`).
  - Subscribes to the estimator's declared input paths.
  - Dispatches incoming input messages to the callback module via
    `c:BB.Estimator.handle_input/2`. Single-input estimators receive the
    bare `%BB.Message{}`; multi-input estimators receive a
    `%{input_name => %BB.Message{}}` map gathered when the driver input
    arrives.
  - For multi-input estimators, enforces `sync_tolerance`: if any
    non-driver input is stale relative to the driver by more than the
    configured tolerance, the dispatch is dropped (with
    `[:bb, :estimator, :dropped]` telemetry) instead of fired with a stale
    snapshot.
  - Publishes each `{output_name, message}` returned from a callback's
    `{:reply, outputs, state}` reply to that output's configured path.
  - Emits `:input`, `:output`, `:latency`, and `:dropped` telemetry.

  Health transitions, lost-detection, and `on_degraded` / `on_lost` /
  `on_recovered` command dispatch are Phase 2 and not handled here yet.

  ## Init args

  The framework supplies the following keys in the start-link init arg.
  Internal keys (double-underscored) are stripped by the server before
  calling `c:BB.Estimator.init/1`; public keys (`:bb`,
  `:estimator_context`) are passed through unchanged so user code can
  read them.

  - `:__callback_module__` - the user's estimator module.
  - `:__estimator_inputs__` - `%{mode: :single | :multi, inputs: [...],
    sync_tolerance_ns: integer() | nil}` describing the input wiring.
  - `:__estimator_outputs__` - `%{output_name => [atom()]}` mapping output
    names to their full pubsub paths.
  - `:bb` - `%{robot: module, path: [atom]}`, the per-process context.
  - `:estimator_context` - the `BB.Estimator.Context.t()` exposed to the
    callback module's `init/1`.

  Plus any user options declared via the estimator's `options_schema/0`.
  """

  use GenServer

  alias BB.Component.OptionsSchema
  alias BB.Estimator.Context
  alias BB.Message
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot.Runtime
  alias BB.Server.ParamResolution

  @framework_keys [:bb, :estimator_context]

  defstruct [
    :callback_module,
    :resolved_opts,
    :raw_opts,
    :param_subscriptions,
    :bb,
    :context,
    :mode,
    :inputs,
    :driver_input,
    :input_name_by_path,
    :sync_tolerance_ns,
    :outputs,
    :last_messages,
    :user_state,
    :latency_budget_ns,
    :lost_after_ns,
    :recover_after,
    :on_degraded,
    :on_lost,
    :on_recovered,
    :health_state,
    :consecutive_ok,
    :lost_timer_ref
  ]

  @type input_spec :: %{
          required(:name) => atom(),
          required(:path) => [atom()],
          required(:driver?) => boolean()
        }

  @type health_state :: :healthy | :degraded | :lost

  @type t :: %__MODULE__{
          callback_module: module(),
          resolved_opts: keyword(),
          raw_opts: keyword(),
          param_subscriptions: %{[atom()] => atom()},
          bb: %{robot: module(), path: [atom()]},
          context: Context.t(),
          mode: :single | :multi,
          inputs: [input_spec()],
          driver_input: atom() | nil,
          input_name_by_path: %{[atom()] => atom()},
          sync_tolerance_ns: integer() | nil,
          outputs: %{atom() => [atom()]},
          last_messages: %{atom() => Message.t()},
          user_state: term(),
          latency_budget_ns: integer() | nil,
          lost_after_ns: integer() | nil,
          recover_after: pos_integer(),
          on_degraded: atom() | nil,
          on_lost: atom() | nil,
          on_recovered: atom() | nil,
          health_state: health_state(),
          consecutive_ok: non_neg_integer(),
          lost_timer_ref: reference() | nil
        }

  @internal_keys [
    :__callback_module__,
    :__estimator_inputs__,
    :__estimator_outputs__,
    :__estimator_health__
  ]

  @doc false
  def start_link(init_arg) do
    GenServer.start_link(__MODULE__, init_arg)
  end

  @doc false
  def start_link(init_arg, opts) do
    GenServer.start_link(__MODULE__, init_arg, opts)
  end

  # ----------------------------------------------------------------------------
  # GenServer lifecycle
  # ----------------------------------------------------------------------------

  @impl GenServer
  def init(init_arg) do
    callback_module = Keyword.fetch!(init_arg, :__callback_module__)
    input_config = Keyword.fetch!(init_arg, :__estimator_inputs__)
    outputs = Keyword.fetch!(init_arg, :__estimator_outputs__)
    health = Keyword.get(init_arg, :__estimator_health__, default_health())

    raw_opts = Keyword.drop(init_arg, @internal_keys)
    bb = Keyword.fetch!(raw_opts, :bb)
    context = Keyword.fetch!(raw_opts, :estimator_context)

    {param_subscriptions, resolved_opts} =
      ParamResolution.resolve_and_subscribe(raw_opts, bb.robot)

    input_name_by_path =
      input_config.inputs
      |> Enum.map(fn input -> {input.path, input.name} end)
      |> Map.new()

    driver_input =
      Enum.find_value(input_config.inputs, fn
        %{driver?: true, name: name} -> name
        _ -> nil
      end)

    Enum.each(input_config.inputs, fn %{path: path} ->
      BB.PubSub.subscribe(bb.robot, path)
    end)

    case OptionsSchema.validate(callback_module, resolved_opts, @framework_keys) do
      {:error, error} ->
        {:stop, error}

      {:ok, resolved_opts} ->
        base = %__MODULE__{
          callback_module: callback_module,
          resolved_opts: resolved_opts,
          raw_opts: raw_opts,
          param_subscriptions: param_subscriptions,
          bb: bb,
          context: context,
          mode: input_config.mode,
          inputs: input_config.inputs,
          driver_input: driver_input,
          input_name_by_path: input_name_by_path,
          sync_tolerance_ns: Map.get(input_config, :sync_tolerance_ns),
          outputs: outputs,
          last_messages: %{},
          latency_budget_ns: health.latency_budget_ns,
          lost_after_ns: health.lost_after_ns,
          recover_after: health.recover_after,
          on_degraded: health.on_degraded,
          on_lost: health.on_lost,
          on_recovered: health.on_recovered,
          health_state: :healthy,
          consecutive_ok: 0,
          lost_timer_ref: nil
        }

        case callback_module.init(resolved_opts) do
          {:ok, user_state} ->
            {:ok, reset_lost_timer(%{base | user_state: user_state})}

          {:ok, user_state, timeout_or_continue} ->
            {:ok, reset_lost_timer(%{base | user_state: user_state}), timeout_or_continue}

          {:stop, reason} ->
            {:stop, reason}

          :ignore ->
            :ignore
        end
    end
  end

  defp default_health do
    %{
      latency_budget_ns: nil,
      lost_after_ns: nil,
      recover_after: 1,
      on_degraded: nil,
      on_lost: nil,
      on_recovered: nil
    }
  end

  # ----------------------------------------------------------------------------
  # Input dispatch
  # ----------------------------------------------------------------------------

  @impl GenServer
  def handle_info({:bb, [:param | param_path], %{payload: %ParameterChanged{}}}, state) do
    case ParamResolution.handle_change(
           param_path,
           state.param_subscriptions,
           state.raw_opts,
           state.bb.robot
         ) do
      {:changed, new_resolved} ->
        with {:ok, new_resolved} <-
               OptionsSchema.validate(state.callback_module, new_resolved, @framework_keys),
             {:ok, new_user_state} <-
               state.callback_module.handle_options(new_resolved, state.user_state) do
          {:noreply, %{state | resolved_opts: new_resolved, user_state: new_user_state}}
        else
          {:stop, reason} -> {:stop, reason, state}
          {:error, error} -> {:stop, error, state}
        end

      :ignored ->
        {:noreply, state}
    end
  end

  def handle_info(:bb_estimator_lost_check, state) do
    {:noreply, transition_to(state, :lost, :lost, nil)}
  end

  def handle_info({:bb, source_path, %Message{} = message}, state) do
    case Map.fetch(state.input_name_by_path, source_path) do
      {:ok, input_name} ->
        emit_input_telemetry(state, source_path)
        state = reset_lost_timer(state)
        dispatch_input(state, input_name, message, source_path)

      :error ->
        delegate_handle_info({:bb, source_path, message}, state)
    end
  end

  def handle_info(msg, state) do
    delegate_handle_info(msg, state)
  end

  defp dispatch_input(%{mode: :single} = state, _input_name, message, source_path) do
    invoke_handle_input(state, message, message, source_path)
  end

  defp dispatch_input(%{mode: :multi} = state, input_name, message, source_path) do
    state = %{state | last_messages: Map.put(state.last_messages, input_name, message)}

    if input_name == state.driver_input do
      maybe_dispatch_multi(state, message, source_path)
    else
      {:noreply, state}
    end
  end

  defp maybe_dispatch_multi(state, driver_message, source_path) do
    snapshot = build_input_snapshot(state, driver_message)

    case check_sync(state, driver_message, snapshot) do
      :ok ->
        invoke_handle_input(state, snapshot, driver_message, source_path)

      {:sync_miss, late_input} ->
        emit_dropped_telemetry(state, late_input, :sync_miss)
        state = transition_to(state, :degraded, :sync_miss, source_path)
        {:noreply, state}
    end
  end

  defp build_input_snapshot(state, driver_message) do
    Enum.reduce(state.inputs, %{}, fn %{name: name}, acc ->
      cond do
        name == state.driver_input -> Map.put(acc, name, driver_message)
        Map.has_key?(state.last_messages, name) -> Map.put(acc, name, state.last_messages[name])
        true -> acc
      end
    end)
  end

  defp check_sync(state, driver_message, snapshot) do
    cond do
      map_size(snapshot) < length(state.inputs) ->
        {:sync_miss, first_missing_input(state.inputs, snapshot)}

      is_nil(state.sync_tolerance_ns) ->
        :ok

      true ->
        check_tolerance(snapshot, driver_message, state.sync_tolerance_ns, state.driver_input)
    end
  end

  defp first_missing_input(inputs, snapshot) do
    Enum.find_value(inputs, fn %{name: name} ->
      if Map.has_key?(snapshot, name), do: nil, else: name
    end)
  end

  defp check_tolerance(snapshot, driver_message, tolerance_ns, driver_name) do
    Enum.reduce_while(snapshot, :ok, fn
      {^driver_name, _msg}, :ok ->
        {:cont, :ok}

      {name, msg}, :ok ->
        gap = abs(driver_message.monotonic_time - msg.monotonic_time)
        if gap <= tolerance_ns, do: {:cont, :ok}, else: {:halt, {:sync_miss, name}}
    end)
  end

  defp invoke_handle_input(state, input, driver_message, source_path) do
    start_time = System.monotonic_time()
    result = state.callback_module.handle_input(input, state.user_state)
    duration_ns = native_to_ns(System.monotonic_time() - start_time)

    state = record_dispatch_outcome(state, duration_ns, source_path)
    handle_callback_result(result, state, driver_message: driver_message, duration: duration_ns)
  end

  defp record_dispatch_outcome(%{latency_budget_ns: nil} = state, _duration, _source_path),
    do: state

  defp record_dispatch_outcome(state, duration_ns, source_path) do
    if duration_ns > state.latency_budget_ns do
      transition_to(state, :degraded, :latency_overrun, source_path)
    else
      handle_in_budget(state)
    end
  end

  defp handle_in_budget(state) do
    case state.health_state do
      :healthy ->
        %{state | consecutive_ok: state.consecutive_ok + 1}

      :degraded ->
        new_count = state.consecutive_ok + 1

        if new_count >= state.recover_after do
          transition_to(%{state | consecutive_ok: 0}, :healthy, :recovered, nil)
        else
          %{state | consecutive_ok: new_count}
        end

      :lost ->
        transition_to(%{state | consecutive_ok: 1}, :degraded, :recovered, nil)
    end
  end

  defp native_to_ns(native), do: System.convert_time_unit(native, :native, :nanosecond)

  # ----------------------------------------------------------------------------
  # Other GenServer callbacks (with output-routing support)
  # ----------------------------------------------------------------------------

  @impl GenServer
  def handle_call(request, from, state) do
    state.callback_module.handle_call(request, from, state.user_state)
    |> handle_call_result(state)
  end

  @impl GenServer
  def handle_cast(request, state) do
    state.callback_module.handle_cast(request, state.user_state)
    |> handle_callback_result(state, [])
  end

  @impl GenServer
  def handle_continue(continue_arg, state) do
    state.callback_module.handle_continue(continue_arg, state.user_state)
    |> handle_callback_result(state, [])
  end

  @impl GenServer
  def terminate(reason, state) do
    state.callback_module.terminate(reason, state.user_state)
  end

  defp delegate_handle_info(msg, state) do
    state.callback_module.handle_info(msg, state.user_state)
    |> handle_callback_result(state, [])
  end

  # ----------------------------------------------------------------------------
  # Reply handling - publishes outputs and folds state.
  # ----------------------------------------------------------------------------

  defp handle_callback_result({:reply, outputs, new_user_state}, state, ctx) do
    publish_outputs(state, outputs, ctx)
    {:noreply, %{state | user_state: new_user_state}}
  end

  defp handle_callback_result(
         {:reply, outputs, new_user_state, timeout_or_continue},
         state,
         ctx
       ) do
    publish_outputs(state, outputs, ctx)
    {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}
  end

  defp handle_callback_result({:noreply, new_user_state}, state, _ctx) do
    {:noreply, %{state | user_state: new_user_state}}
  end

  defp handle_callback_result({:noreply, new_user_state, timeout_or_continue}, state, _ctx) do
    {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}
  end

  defp handle_callback_result({:stop, reason, new_user_state}, state, _ctx) do
    {:stop, reason, %{state | user_state: new_user_state}}
  end

  defp handle_call_result({:reply, reply, outputs, new_user_state}, state)
       when is_list(outputs) do
    publish_outputs(state, outputs, [])
    {:reply, reply, %{state | user_state: new_user_state}}
  end

  defp handle_call_result({:reply, reply, new_user_state}, state) do
    {:reply, reply, %{state | user_state: new_user_state}}
  end

  defp handle_call_result({:reply, reply, new_user_state, timeout_or_continue}, state) do
    {:reply, reply, %{state | user_state: new_user_state}, timeout_or_continue}
  end

  defp handle_call_result({:noreply, new_user_state}, state) do
    {:noreply, %{state | user_state: new_user_state}}
  end

  defp handle_call_result({:noreply, new_user_state, timeout_or_continue}, state) do
    {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}
  end

  defp handle_call_result({:stop, reason, new_user_state}, state) do
    {:stop, reason, %{state | user_state: new_user_state}}
  end

  defp handle_call_result({:stop, reason, reply, new_user_state}, state) do
    {:stop, reason, reply, %{state | user_state: new_user_state}}
  end

  defp publish_outputs(state, outputs, ctx) do
    Enum.each(outputs, fn {output_name, %Message{} = message} ->
      case Map.fetch(state.outputs, output_name) do
        {:ok, path} ->
          BB.PubSub.publish(state.bb.robot, path, message)
          emit_output_telemetry(state, output_name, message)
          maybe_emit_latency_telemetry(state, output_name, message, ctx)

        :error ->
          raise ArgumentError,
                "Estimator #{inspect(state.callback_module)} emitted unknown output " <>
                  "#{inspect(output_name)}. Declared outputs: " <>
                  inspect(Map.keys(state.outputs))
      end
    end)
  end

  # ----------------------------------------------------------------------------
  # Telemetry
  # ----------------------------------------------------------------------------

  defp emit_input_telemetry(state, source_path) do
    :telemetry.execute(
      [:bb, :estimator, :input],
      %{count: 1},
      %{
        robot: state.bb.robot,
        estimator: List.last(state.bb.path),
        source_path: source_path
      }
    )
  end

  defp emit_output_telemetry(state, output_name, message) do
    :telemetry.execute(
      [:bb, :estimator, :output],
      %{count: 1},
      %{
        robot: state.bb.robot,
        estimator: List.last(state.bb.path),
        output: output_name,
        payload_module: message.payload.__struct__
      }
    )
  end

  defp maybe_emit_latency_telemetry(state, output_name, _message, ctx) do
    with driver_message when not is_nil(driver_message) <- Keyword.get(ctx, :driver_message),
         duration when is_integer(duration) <- Keyword.get(ctx, :duration) do
      :telemetry.execute(
        [:bb, :estimator, :latency],
        %{
          duration: duration,
          input_to_output: System.monotonic_time() - driver_message.monotonic_time
        },
        %{
          robot: state.bb.robot,
          estimator: List.last(state.bb.path),
          output: output_name
        }
      )
    end
  end

  defp emit_dropped_telemetry(state, source_input, reason) do
    :telemetry.execute(
      [:bb, :estimator, :dropped],
      %{count: 1},
      %{
        robot: state.bb.robot,
        estimator: List.last(state.bb.path),
        source_input: source_input,
        reason: reason
      }
    )
  end

  defp emit_transition_telemetry(state, from, to, reason) do
    :telemetry.execute(
      [:bb, :estimator, :transition],
      %{count: 1},
      %{
        robot: state.bb.robot,
        estimator: List.last(state.bb.path),
        from: from,
        to: to,
        reason: reason
      }
    )
  end

  # ----------------------------------------------------------------------------
  # Health transitions
  # ----------------------------------------------------------------------------

  defp transition_to(state, new_health_state, reason, source_path) do
    previous = state.health_state

    if previous == new_health_state do
      state
    else
      emit_transition_telemetry(state, previous, new_health_state, reason)
      dispatch_transition_command(state, previous, new_health_state, reason, source_path)
      %{state | health_state: new_health_state, consecutive_ok: 0}
    end
  end

  defp dispatch_transition_command(state, previous, new_state, reason, source_path) do
    command_name = command_for_state(state, new_state)

    if command_name do
      args = %{
        estimator: List.last(state.bb.path),
        reason: reason,
        source_path: source_path,
        previous_state: previous,
        new_state: new_state
      }

      _ = Runtime.execute(state.bb.robot, command_name, args)
    end

    :ok
  end

  defp command_for_state(state, :degraded), do: state.on_degraded
  defp command_for_state(state, :lost), do: state.on_lost
  defp command_for_state(state, :healthy), do: state.on_recovered

  # ----------------------------------------------------------------------------
  # Lost-detection timer
  # ----------------------------------------------------------------------------

  defp reset_lost_timer(%{lost_after_ns: nil} = state), do: state

  defp reset_lost_timer(state) do
    if is_reference(state.lost_timer_ref) do
      Process.cancel_timer(state.lost_timer_ref)
    end

    timeout_ms = max(div(state.lost_after_ns, 1_000_000), 1)
    ref = Process.send_after(self(), :bb_estimator_lost_check, timeout_ms)
    %{state | lost_timer_ref: ref}
  end
end
