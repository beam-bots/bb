# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator do
  @moduledoc """
  Behaviour for state estimators in the BB framework.

  Estimators consume one or more input message streams and publish derived
  state. The same contract covers within-sensor fusion (e.g. an AHRS algorithm
  combining gyro and accelerometer from a single IMU into orientation) and
  cross-sensor fusion (e.g. an EKF combining IMU and wheel odometry into a
  base pose).

  Estimators are declared inline in the DSL using the `estimator` entity,
  which may nest inside either a `sensor` (single-input form, frame inherited)
  or a `link` (cross-sensor form, frame = link).

  ## Usage

  The `use BB.Estimator` macro sets up your module as an estimator callback
  module. Your module is NOT a GenServer - the framework provides a wrapper
  GenServer (`BB.Estimator.Server`) that delegates to your callbacks and
  routes returned messages to the appropriate pubsub paths.

  ### Required Callbacks

  - `init/1` - Initialise estimator state from resolved options
  - `handle_input/2` - Consume an input message and optionally emit outputs

  ### Optional Callbacks

  - `handle_options/2` - React to parameter changes at runtime
  - `handle_info/2`, `handle_call/3`, `handle_cast/2`, `handle_continue/2`,
    `terminate/2` - Standard GenServer-style callbacks
  - `options_schema/0` - Define accepted configuration options

  ### Reply Shape

  Unlike sensors and controllers, estimators emit messages by returning them
  from their callbacks rather than calling `BB.publish/3` directly. Each
  callback that can emit messages accepts a `{:reply, outputs, state}` reply,
  where `outputs` is a list of `{output_name, %BB.Message{}}` tuples.

  - `output_name` is either an atom matching an `output :name` block on the
    estimator, or the conventional `:out` atom for single-output estimators.
  - Returning an empty list emits nothing - useful for accumulators that
    consume many inputs before producing one output.

  ### Init Context

  The framework injects an `:estimator_context` option carrying a
  `BB.Estimator.Context` struct alongside the existing `:bb` option. The
  context provides the target frame, the static transforms from each input's
  source frame to the target frame, and the estimator's full path.

      defmodule MyEstimator do
        use BB.Estimator,
          options_schema: [
            gain: [type: :float, default: 0.1, doc: "Filter gain"]
          ]

        @impl BB.Estimator
        def init(opts) do
          gain = Keyword.fetch!(opts, :gain)
          context = Keyword.fetch!(opts, :estimator_context)
          {:ok, %{gain: gain, transforms: context.transforms}}
        end

        @impl BB.Estimator
        def handle_input(%BB.Message{} = msg, state) do
          out_msg = compute(msg, state)
          {:reply, [out: out_msg], state}
        end
      end

  ### Single vs Multi-Input Dispatch

  For single-input estimators (sensor-nested, or link-nested with one declared
  `input`), `handle_input/2` receives a single `%BB.Message{}`. For multi-input
  estimators, it receives a map of `%{input_name => %BB.Message{}}` keyed by
  the `input` declaration name, populated by the framework when the driver
  input arrives.

  ### Auto-injected Options

  The `:bb` and `:estimator_context` options are auto-injected by the framework
  and should NOT appear in `options_schema/0`. The `:bb` option contains
  `%{robot: module, path: [atom]}`.
  """

  alias BB.Estimator.Context
  alias BB.Message

  @typedoc "An emitted output: `{output_name, message}`. `:out` is the conventional name for single-output estimators."
  @type output :: {atom(), Message.t()}

  @typedoc "Input delivered to `handle_input/2`. Single-input estimators receive the bare message; multi-input estimators receive a map keyed by input name."
  @type input :: Message.t() | %{atom() => Message.t()}

  # ----------------------------------------------------------------------------
  # Behaviour
  # ----------------------------------------------------------------------------

  @doc """
  Initialise estimator state from resolved options.

  Called with options after parameter references have been resolved. The
  framework-injected options are:

  - `:bb` - `%{robot: module, path: [atom]}`
  - `:estimator_context` - a `BB.Estimator.Context.t()`

  Return `{:ok, state}` or `{:ok, state, timeout_or_continue}` on success,
  `{:stop, reason}` to abort startup, or `:ignore` to skip this estimator.
  """
  @callback init(opts :: keyword()) ::
              {:ok, state :: term()}
              | {:ok, state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term()}
              | :ignore

  @doc """
  Consume an input message (or a fanned-in bundle of inputs) and optionally
  emit output messages.

  Single-input estimators receive a `%BB.Message{}`. Multi-input estimators
  receive a `%{input_name => %BB.Message{}}` map, gathered by the framework
  when the configured driver input arrives.

  The `{:reply, outputs, state}` return shape publishes each `{name, message}`
  in `outputs` to the corresponding output path. Returning `{:noreply, state}`
  or `{:reply, [], state}` emits nothing.
  """
  @callback handle_input(input(), state :: term()) ::
              {:reply, [output()], new_state :: term()}
              | {:reply, [output()], new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle parameter changes at runtime.

  Called when a referenced parameter changes. The `new_opts` contain all
  options with the updated parameter value(s) resolved.

  Return `{:ok, new_state}` to update state, or `{:stop, reason}` to shut
  down.
  """
  @callback handle_options(new_opts :: keyword(), state :: term()) ::
              {:ok, new_state :: term()} | {:stop, reason :: term()}

  @doc """
  Handle synchronous calls.

  Same semantics as `c:GenServer.handle_call/3`, extended with a
  `{:reply, reply, outputs, state}` form that lets a call response also
  publish output messages.
  """
  @callback handle_call(request :: term(), from :: GenServer.from(), state :: term()) ::
              {:reply, reply :: term(), new_state :: term()}
              | {:reply, reply :: term(), new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:reply, reply :: term(), [output()], new_state :: term()}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}
              | {:stop, reason :: term(), reply :: term(), new_state :: term()}

  @doc """
  Handle asynchronous casts.

  Same semantics as `c:GenServer.handle_cast/2`. May emit outputs via the
  `{:reply, outputs, state}` form.
  """
  @callback handle_cast(request :: term(), state :: term()) ::
              {:reply, [output()], new_state :: term()}
              | {:reply, [output()], new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle all other messages.

  Same semantics as `c:GenServer.handle_info/2`. May emit outputs via the
  `{:reply, outputs, state}` form - useful for estimators that emit on a
  timer.
  """
  @callback handle_info(msg :: term(), state :: term()) ::
              {:reply, [output()], new_state :: term()}
              | {:reply, [output()], new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle continue instructions.

  Same semantics as `c:GenServer.handle_continue/2`. May emit outputs.
  """
  @callback handle_continue(continue_arg :: term(), state :: term()) ::
              {:reply, [output()], new_state :: term()}
              | {:reply, [output()], new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Clean up before termination.

  Same semantics as `c:GenServer.terminate/2`.
  """
  @callback terminate(reason :: term(), state :: term()) :: term()

  @doc """
  Returns the options schema for this estimator.

  The schema should NOT include the `:bb` or `:estimator_context` options -
  both are auto-injected by the framework.
  """
  @callback options_schema() :: Spark.Options.t()

  @optional_callbacks [
    handle_options: 2,
    handle_call: 3,
    handle_cast: 2,
    handle_info: 2,
    handle_continue: 2,
    terminate: 2
  ]

  alias BB.Component.OptionsSchema

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      @behaviour BB.Estimator

      @impl BB.Estimator
      def handle_options(_new_opts, state), do: {:ok, state}

      @impl BB.Estimator
      def handle_call(_request, _from, state), do: {:reply, {:error, :not_implemented}, state}

      @impl BB.Estimator
      def handle_cast(_request, state), do: {:noreply, state}

      @impl BB.Estimator
      def handle_info(_msg, state), do: {:noreply, state}

      @impl BB.Estimator
      def handle_continue(_continue_arg, state), do: {:noreply, state}

      @impl BB.Estimator
      def terminate(_reason, _state), do: :ok

      defoverridable handle_options: 2,
                     handle_call: 3,
                     handle_cast: 2,
                     handle_info: 2,
                     handle_continue: 2,
                     terminate: 2

      unquote(OptionsSchema.inject(BB.Estimator, schema_opts))
    end
  end

  # ----------------------------------------------------------------------------
  # Re-exports for users that want to type-spec their estimator state.
  # ----------------------------------------------------------------------------

  @typedoc "The framework-provided init context, delivered as the `:estimator_context` opt."
  @type context :: Context.t()
end
