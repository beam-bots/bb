# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.Server do
  @moduledoc """
  Wrapper GenServer for estimator callback modules.

  Responsibilities:

  - Resolves parameter references in opts at startup and on parameter
    changes (mirroring `BB.Sensor.Server` / `BB.Controller.Server`).
  - Subscribes to each input through its `BB.Estimator.Source`, and asks
    the source for the next input each time it has finished with the last.
    Inputs are `BB.PubSub` messages unless a sensor-nested estimator's
    parent sensor names another source.
  - Rejects any input produced on a node other than the local one before
    its timestamp is compared against anything, since monotonic clocks are
    node-local. A rejected input is dropped with reason `:cross_node` and
    is not retained, so a multi-input alias whose newest input came from
    another node stays missing until a local one arrives.
  - Dispatches inputs to the callback module via
    `c:BB.Estimator.handle_input/2`. Single-input estimators receive the
    bare input - a `%BB.Message{}` from PubSub, whatever the source yields
    otherwise; multi-input estimators receive a
    `%{input_name => %BB.Message{}}` map gathered when the driver input
    arrives.
  - Enforces `max_input_age`: an input older than its budget, by the
    timestamp its source supplied, is discarded before `handle_input/2` is
    called, and the estimator transitions to `:degraded` with reason
    `:stale_input`. Retained non-driver inputs are re-checked when the
    driver arrives, since they can age past their budget while they wait.
    This is independent of `latency_budget`, which measures callback
    execution time.
  - Stops with `{:shutdown, {:source_ended, input_name, reason}}` when a
    source reports it will deliver nothing more, so the supervisor starts
    it again with a fresh subscription.
  - For multi-input estimators, enforces `sync_tolerance`: if any
    non-driver input is stale relative to the driver by more than the
    configured tolerance, the dispatch is dropped (with
    `[:bb, :estimator, :dropped]` telemetry) instead of fired with a stale
    snapshot.
  - Publishes each `{output_name, message}` returned from a callback's
    `{:reply, outputs, state}` reply to that output's configured path.
  - Runs the `:healthy` / `:degraded` / `:lost` health state machine and
    dispatches the configured `on_degraded` / `on_lost` / `on_recovered`
    commands on each transition. For multi-input estimators only driver
    arrivals reset the `lost_after` timer, so a live auxiliary stream
    cannot mask a driver that has stopped publishing.
  - Emits `:input`, `:output`, `:latency`, `:dropped`, and `:transition`
    telemetry. Every timing measurement is in nanoseconds, the same unit as
    the `monotonic_time` a source supplies.

  ## Init args

  The framework supplies the following keys in the start-link init arg.
  Internal keys (double-underscored) are stripped by the server before
  calling `c:BB.Estimator.init/1`; public keys (`:bb`,
  `:estimator_context`) are passed through unchanged so user code can
  read them.

  - `:__callback_module__` - the user's estimator module.
  - `:__estimator_inputs__` - `%{mode: :single | :multi, inputs: [...],
    sync_tolerance_ns: integer() | nil}` describing the input wiring. Each
    input spec carries `:name`, `:path`, `:driver?`, a resolved
    `:max_input_age_ns` and the `{source_module, args}` it is read through.
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
    :sources,
    :max_input_age_ns,
    :sync_tolerance_ns,
    :outputs,
    :last_inputs,
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
          required(:driver?) => boolean(),
          optional(:max_input_age_ns) => integer() | nil,
          required(:source) => {module(), keyword()}
        }

  @type source :: %{
          name: atom(),
          path: [atom()],
          module: module(),
          state: BB.Estimator.Source.state()
        }

  @type arrival :: %{input: term(), monotonic_time: integer()}

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
          sources: [source()],
          max_input_age_ns: %{atom() => integer() | nil},
          sync_tolerance_ns: integer() | nil,
          outputs: %{atom() => [atom()]},
          last_inputs: %{atom() => arrival()},
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

    max_input_age_ns =
      Map.new(input_config.inputs, fn input ->
        {input.name, Map.get(input, :max_input_age_ns)}
      end)

    driver_input =
      Enum.find_value(input_config.inputs, fn
        %{driver?: true, name: name} -> name
        _ -> nil
      end)

    with {:ok, sources} <- subscribe_sources(input_config.inputs, bb.robot),
         {:ok, resolved_opts} <-
           OptionsSchema.validate(callback_module, resolved_opts, @framework_keys) do
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
        sources: sources,
        max_input_age_ns: max_input_age_ns,
        sync_tolerance_ns: Map.get(input_config, :sync_tolerance_ns),
        outputs: outputs,
        last_inputs: %{},
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
          Enum.each(sources, &request_input/1)
          {:ok, reset_lost_timer(%{base | user_state: user_state})}

        {:ok, user_state, timeout_or_continue} ->
          Enum.each(sources, &request_input/1)
          {:ok, reset_lost_timer(%{base | user_state: user_state}), timeout_or_continue}

        {:stop, reason} ->
          {:stop, reason}

        :ignore ->
          :ignore
      end
    else
      {:error, error} -> {:stop, error}
    end
  end

  defp subscribe_sources(inputs, robot) do
    Enum.reduce_while(inputs, {:ok, []}, fn input, {:ok, sources} ->
      {module, args} = input.source

      case module.subscribe(robot, input.path, args) do
        {:ok, source_state} ->
          source = %{name: input.name, path: input.path, module: module, state: source_state}
          {:cont, {:ok, sources ++ [source]}}

        {:error, error} ->
          {:halt, {:error, error}}
      end
    end)
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

  def handle_info(msg, state) do
    case source_event(state.sources, msg) do
      nil -> delegate_handle_info(msg, state)
      {source, event} -> handle_source_event(state, source, event)
    end
  end

  defp source_event(sources, msg) do
    Enum.find_value(sources, fn source ->
      case source.module.input_event(source.state, msg) do
        nil -> nil
        event -> {source, event}
      end
    end)
  end

  defp handle_source_event(state, source, {:input, input, monotonic_time, input_node}) do
    emit_input_telemetry(state, source.path)
    arrival = %{input: input, monotonic_time: monotonic_time}
    result = accept_input(state, source.name, arrival, input_node, source.path)
    request_input(source)
    result
  end

  defp handle_source_event(state, _source, :no_input), do: {:noreply, state}

  defp handle_source_event(state, source, :spent) do
    request_input(source)
    {:noreply, state}
  end

  defp handle_source_event(state, source, {:ended, reason}) do
    {:stop, {:shutdown, {:source_ended, source.name, reason}}, state}
  end

  defp request_input(source), do: source.module.request(source.state)

  defp accept_input(state, input_name, arrival, input_node, source_path) do
    case check_intake(state, input_name, arrival, input_node) do
      :ok ->
        state = maybe_reset_lost_timer(state, input_name)
        dispatch_input(state, input_name, arrival, source_path)

      {:reject, reason} ->
        emit_dropped_telemetry(state, input_name, reason)
        {:noreply, transition_to(state, :degraded, reason, source_path)}
    end
  end

  # Monotonic clocks are node-local, so a remote input's `monotonic_time`
  # cannot be compared with anything this node has recorded.
  defp check_intake(_state, _input_name, _arrival, input_node) when input_node != node(),
    do: {:reject, :cross_node}

  defp check_intake(state, input_name, arrival, _input_node) do
    if stale?(state, input_name, arrival), do: {:reject, :stale_input}, else: :ok
  end

  defp stale?(state, input_name, arrival) do
    case Map.get(state.max_input_age_ns, input_name) do
      nil -> false
      max_age_ns -> System.monotonic_time(:nanosecond) - arrival.monotonic_time > max_age_ns
    end
  end

  defp dispatch_input(%{mode: :single} = state, _input_name, arrival, source_path) do
    invoke_handle_input(state, arrival.input, arrival, source_path)
  end

  defp dispatch_input(%{mode: :multi} = state, input_name, arrival, source_path) do
    state = %{state | last_inputs: Map.put(state.last_inputs, input_name, arrival)}

    if input_name == state.driver_input do
      maybe_dispatch_multi(state, arrival, source_path)
    else
      {:noreply, state}
    end
  end

  defp maybe_dispatch_multi(state, driver_arrival, source_path) do
    snapshot = build_input_snapshot(state, driver_arrival)

    case check_sync(state, driver_arrival, snapshot) do
      :ok ->
        inputs = Map.new(snapshot, fn {name, arrival} -> {name, arrival.input} end)
        invoke_handle_input(state, inputs, driver_arrival, source_path)

      {reason, rejected_input} ->
        emit_dropped_telemetry(state, rejected_input, reason)
        {:noreply, transition_to(state, :degraded, reason, source_path)}
    end
  end

  defp build_input_snapshot(state, driver_arrival) do
    Enum.reduce(state.inputs, %{}, fn %{name: name}, acc ->
      cond do
        name == state.driver_input -> Map.put(acc, name, driver_arrival)
        Map.has_key?(state.last_inputs, name) -> Map.put(acc, name, state.last_inputs[name])
        true -> acc
      end
    end)
  end

  defp check_sync(state, driver_arrival, snapshot) do
    with :ok <- check_complete(state, snapshot),
         :ok <- check_freshness(state, snapshot) do
      check_tolerance(snapshot, driver_arrival, state.sync_tolerance_ns, state.driver_input)
    end
  end

  defp check_complete(state, snapshot) when map_size(snapshot) < length(state.inputs),
    do: {:sync_miss, first_missing_input(state.inputs, snapshot)}

  defp check_complete(_state, _snapshot), do: :ok

  # A retained non-driver input can age past its budget while it waits for
  # the next driver arrival, so the snapshot is re-checked at dispatch.
  defp check_freshness(state, snapshot) do
    Enum.reduce_while(snapshot, :ok, fn {name, arrival}, :ok ->
      if stale?(state, name, arrival),
        do: {:halt, {:stale_input, name}},
        else: {:cont, :ok}
    end)
  end

  defp first_missing_input(inputs, snapshot) do
    Enum.find_value(inputs, fn %{name: name} ->
      if Map.has_key?(snapshot, name), do: nil, else: name
    end)
  end

  defp check_tolerance(_snapshot, _driver_arrival, nil, _driver_name), do: :ok

  defp check_tolerance(snapshot, driver_arrival, tolerance_ns, driver_name) do
    Enum.reduce_while(snapshot, :ok, fn
      {^driver_name, _arrival}, :ok ->
        {:cont, :ok}

      {name, arrival}, :ok ->
        gap = abs(driver_arrival.monotonic_time - arrival.monotonic_time)
        if gap <= tolerance_ns, do: {:cont, :ok}, else: {:halt, {:sync_miss, name}}
    end)
  end

  defp invoke_handle_input(state, input, driver_arrival, source_path) do
    start_time_ns = System.monotonic_time(:nanosecond)
    result = state.callback_module.handle_input(input, state.user_state)
    duration_ns = System.monotonic_time(:nanosecond) - start_time_ns

    state = record_dispatch_outcome(state, duration_ns, source_path)
    handle_callback_result(result, state, driver_arrival: driver_arrival, duration: duration_ns)
  end

  defp record_dispatch_outcome(%{latency_budget_ns: nil} = state, _duration_ns, _source_path),
    do: handle_in_budget(state)

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
    with driver_arrival when not is_nil(driver_arrival) <- Keyword.get(ctx, :driver_arrival),
         duration when is_integer(duration) <- Keyword.get(ctx, :duration) do
      :telemetry.execute(
        [:bb, :estimator, :latency],
        %{
          duration: duration,
          input_to_output: System.monotonic_time(:nanosecond) - driver_arrival.monotonic_time
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

  defp maybe_reset_lost_timer(%{mode: :single} = state, _input_name), do: reset_lost_timer(state)

  defp maybe_reset_lost_timer(%{driver_input: input_name} = state, input_name),
    do: reset_lost_timer(state)

  defp maybe_reset_lost_timer(state, _input_name), do: state

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
