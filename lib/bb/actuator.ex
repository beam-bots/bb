# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator do
  @moduledoc """
  Behaviour and API for actuators in the BB framework.

  This module serves two purposes:

  1. **Behaviour** - Defines callbacks for actuator implementations
  2. **API** - Provides functions for sending commands to actuators

  ## Behaviour

  Actuators receive position/velocity/effort commands and drive hardware.
  They must implement the `init/1` and `disarm/1` callbacks.

  ## Usage

  The `use BB.Actuator` macro sets up your module as an actuator callback module.
  Your module is NOT a GenServer - the framework provides a wrapper GenServer
  (`BB.Actuator.Server`) that delegates to your callbacks.

  ### Required Callbacks

  - `init/1` - Initialise actuator state from resolved options
  - `handle_command/2` - Act on an inbound command
  - `disarm/1` - Make hardware safe (called without GenServer state)

  ### Optional Callbacks

  - `capabilities/1` - Declare that the driver reads position, velocity or
    effort back from the hardware. Without it the framework assumes it doesn't,
    and warns that the joint needs a sensor
  - `handle_options/2` - React to parameter changes at runtime
  - `handle_call/3`, `handle_cast/2`, `handle_info/2` - Standard GenServer-style
    callbacks, for the driver's own traffic
  - `handle_continue/2`, `terminate/2` - Lifecycle callbacks
  - `options_schema/0` - Define accepted configuration options

  ### Options Schema

  If your actuator accepts configuration options, pass them via `:options_schema`:

      defmodule MyServoActuator do
        use BB.Actuator,
          options_schema: [
            channel: [type: {:in, 0..15}, required: true, doc: "PWM channel"],
            controller: [type: :atom, required: true, doc: "Controller name"]
          ]

        @impl BB.Actuator
        def init(opts) do
          channel = Keyword.fetch!(opts, :channel)
          bb = Keyword.fetch!(opts, :bb)
          {:ok, %{channel: channel, bb: bb}}
        end

        @impl BB.Actuator
        def disarm(opts) do
          MyHardware.disable(opts[:controller], opts[:channel])
          :ok
        end

        @impl BB.Actuator
        def handle_command(%BB.Message{payload: %Command.Position{} = cmd}, state) do
          MyHardware.write(state.channel, cmd.position)
          {:noreply, state}
        end
      end

  For actuators that don't need configuration, omit `:options_schema`:

      defmodule SimpleActuator do
        use BB.Actuator

        @impl BB.Actuator
        def init(opts) do
          {:ok, %{bb: opts[:bb]}}
        end

        @impl BB.Actuator
        def handle_command(_message, state), do: {:noreply, state}

        @impl BB.Actuator
        def disarm(_opts), do: :ok
      end

  ### Parameter References

  Options can reference parameters for runtime-adjustable configuration:

      actuator :motor, {MyMotor, max_effort: param([:motion, :max_effort])}

  When the parameter changes, `handle_options/2` is called with the new resolved
  options. Override it to update your state accordingly.

  ### Auto-injected Options

  The `:bb` option is automatically provided and should NOT be included in your
  `options_schema`. It contains `%{robot: module, path: [atom]}`.

  ### Safety Registration

  Safety registration is automatic - the framework registers your module with
  `BB.Safety` using the resolved options. You don't need to call `BB.Safety.register`
  manually.

  ## API

  ### Delivery Methods

  Every command function - `set_position/4`, `set_velocity/4`, `set_effort/4`,
  `follow_trajectory/4`, `stop/3` and `hold/3` - takes the same `:delivery`
  option, as does `BB.Motion`:

  - **`:pubsub`** - The command is published to `[:actuator | path]` so
    orchestration and logging can observe it, *and* delivered to the actuator
    by a call. Returns `:ok` or `{:error, reason}`, so a caller finds out that
    a joint isn't moving rather than assuming it is.

  - **`:broadcast`** - Published to `[:actuator | path]` and nothing more. The
    actuator picks the command up through its own subscription, alongside every
    other observer. The caller doesn't wait, and always gets `:ok`.

  - **`:direct`** - Sent via `BB.Process.cast`, publishing nothing. The lowest
    latency of the three, for time-critical control, at the price of a refusal
    nobody sees. Always returns `:ok`.

  The default differs per command, and each function states its own:
  `set_position/4` defaults to `:pubsub`; the other five default to
  `:broadcast`, because they sit on control paths where blocking the caller
  would be the more surprising behaviour.

  All three converge on `c:handle_command/2`. Which transport a caller chose is
  not something a driver has to know about, and choosing one cannot skip the
  checks `BB.Actuator.Server` applies on the way in.

  ### Hearing about refusals without waiting

  `:broadcast` and `:direct` both return `:ok` whether the actuator took the
  command or threw it out, so neither can be matched on for failure. Under
  `:direct`, `reply_on_reject?: true` closes that gap: the actuator sends the
  calling process

      {:bb, :command_rejected, actuator_name, command_id, error}

  when it refuses, and nothing at all when it accepts. `error` is the same
  `BB.Error` struct `:pubsub` would have returned. `command_id` is whatever the
  caller passed as `:command_id`, and is `nil` if it passed none - correlating
  a reply with the command that caused it means setting one.

      ref = make_ref()

      :ok =
        BB.Actuator.set_position(MyRobot, :servo, 1.57,
          delivery: :direct,
          command_id: ref,
          reply_on_reject?: true
        )

      receive do
        {:bb, :command_rejected, :servo, ^ref, error} -> Logger.error(Exception.message(error))
      after
        0 -> :ok
      end

  The reply goes to whichever process made the call, so a command sent from
  inside a `Task` is answered to the task and not to whoever started it.

  The option is `:direct` only, and raises `Spark.Options.ValidationError`
  anywhere else: `:pubsub` returns the refusal already, and a `:broadcast`
  command has no one caller to name.

  ### Option validation

  Each command validates its options against a `Spark.Options` schema before
  sending anything, so a misspelled `velocty:` is a raised
  `Spark.Options.ValidationError` rather than a hint that quietly does nothing.
  The accepted keys are listed under each function.

  ### Arm epochs

  Every function here stamps its outgoing command with the robot's current arm
  epoch, and an actuator refuses a command whose epoch is missing or belongs to
  an earlier arming session — so a command cannot outlive the arming session
  that authorised it. See `BB.Safety.epoch/1`.

  Building a command and delivering it yourself, with `BB.publish/4` or
  `BB.cast/3`, skips the stamping and the command is refused. Either use the
  functions here, or stamp it yourself:

      {:ok, epoch} = BB.Safety.epoch(MyRobot)
      message = %{Message.new!(Command.Position, :servo, position: 1.57) | arm_epoch: epoch}

  ### Addressing

  Every function accepts either the actuator's unique name or its full path
  through the topology. Names are resolved against the robot with
  `BB.Robot.actuator_path/2`, so the two are interchangeable:

      BB.Actuator.set_position(MyRobot, :shoulder_servo, 1.57)
      BB.Actuator.set_position(MyRobot, [:base_link, :shoulder, :shoulder_servo], 1.57)

  Naming an actuator the robot doesn't have raises `ArgumentError` rather than
  publishing to a topic nothing is listening on.

  ### Examples

      # Published for observers, acknowledged by the actuator
      :ok = BB.Actuator.set_position(MyRobot, :shoulder_servo, 1.57)

      # Published for observers, but not waited on
      :ok = BB.Actuator.set_velocity(MyRobot, :shoulder_servo, 0.25)

      # Fire-and-forget (for time-critical control)
      :ok = BB.Actuator.set_position(MyRobot, :shoulder_servo, 1.57, delivery: :direct)
  """

  # ----------------------------------------------------------------------------
  # Behaviour
  # ----------------------------------------------------------------------------

  @doc """
  Initialise actuator state from resolved options.

  Called with options after parameter references have been resolved.
  The `:bb` key contains `%{robot: module, path: [atom]}`.

  Return `{:ok, state}` or `{:ok, state, timeout_or_continue}` on success,
  `{:stop, reason}` to abort startup, or `:ignore` to skip this actuator.
  """
  @callback init(opts :: keyword()) ::
              {:ok, state :: term()}
              | {:ok, state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term()}
              | :ignore

  @doc """
  Act on an inbound command.

  Called for every command that reaches this actuator, whichever transport
  delivered it. By the time it arrives, `BB.Actuator.Server` has checked that
  the robot is armed and translated the payload from joint-space into
  motor-space, so the values are ready to write to hardware.

  The reply is used only by callers that wait for one - those that chose
  `delivery: :pubsub`. Returning `{:noreply, state}` replies `{:ok, :accepted}`
  to such a caller, which the send functions report as `:ok`. Only an
  `{:error, reason}` reply tells a caller its command was refused, and under
  `:broadcast` or `:direct` it is discarded along with the rest.

      @impl BB.Actuator
      def handle_command(%BB.Message{payload: %Command.Position{} = cmd}, state) do
        MyHardware.write(state.channel, cmd.position)
        {:noreply, state}
      end

  Commands the driver doesn't implement should fall through to a catch-all
  clause rather than crashing the actuator - a `Command.Trajectory` sent to a
  position-only servo is a caller error, not a hardware fault.

  > #### `Command.Stop` is a motion command, not a safety one {: .info}
  >
  > `Stop` means *cease travelling and become passive* — it's the counterpart to
  > `Command.Hold`, which maintains position and resists external force. Its
  > `:decelerate` mode makes that plain: nothing that slows down smoothly is an
  > emergency stop.
  >
  > Making hardware safe is `c:disarm/1`, which is robot-wide, runs without
  > GenServer state, and leaves the robot unable to move until re-armed. Don't
  > reach for `Stop` to do that job.
  >
  > If you declare `Stop` in `c:command_payloads/1` — and the default does —
  > give it a clause that genuinely stops driving, rather than letting a
  > catch-all swallow it and report success while the joint keeps moving:
  >
  > ```elixir
  > def handle_command(%BB.Message{payload: %Command.Stop{}}, state) do
  >   MyHardware.cut_drive(state.channel)
  >   {:noreply, state}
  > end
  > ```
  >
  """
  @callback handle_command(command :: BB.Message.t(), state :: term()) ::
              {:reply, reply :: term(), new_state :: term()}
              | {:reply, reply :: term(), new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Make the hardware safe.

  Called with the opts provided at registration. Must work without GenServer state.
  This callback is required for actuators since they control physical hardware.
  """
  @callback disarm(opts :: keyword()) :: :ok | {:error, term()}

  @doc """
  Handle parameter changes at runtime.

  Called when a referenced parameter changes. The `new_opts` contain all options
  with the updated parameter value(s) resolved.

  Return `{:ok, new_state}` to update state, or `{:stop, reason}` to shut down.
  """
  @callback handle_options(new_opts :: keyword(), state :: term()) ::
              {:ok, new_state :: term()} | {:stop, reason :: term()}

  @doc """
  Handle synchronous calls other than commands.

  Same semantics as `c:GenServer.handle_call/3`. Commands arrive at
  `c:handle_command/2` regardless of transport.
  """
  @callback handle_call(request :: term(), from :: GenServer.from(), state :: term()) ::
              {:reply, reply :: term(), new_state :: term()}
              | {:reply, reply :: term(), new_state :: term(),
                 timeout() | :hibernate | {:continue, term()}}
              | {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}
              | {:stop, reason :: term(), reply :: term(), new_state :: term()}

  @doc """
  Handle asynchronous casts other than commands.

  Same semantics as `c:GenServer.handle_cast/2`. Commands arrive at
  `c:handle_command/2` regardless of transport.
  """
  @callback handle_cast(request :: term(), state :: term()) ::
              {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle all other messages.

  Same semantics as `c:GenServer.handle_info/2`. Messages from topics the
  driver subscribed to itself arrive here untouched - the server neither
  transforms nor intercepts them. Commands addressed to this actuator arrive
  at `c:handle_command/2` instead.
  """
  @callback handle_info(msg :: term(), state :: term()) ::
              {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Handle continue instructions.

  Same semantics as `c:GenServer.handle_continue/2`.
  """
  @callback handle_continue(continue_arg :: term(), state :: term()) ::
              {:noreply, new_state :: term()}
              | {:noreply, new_state :: term(), timeout() | :hibernate | {:continue, term()}}
              | {:stop, reason :: term(), new_state :: term()}

  @doc """
  Clean up before termination.

  Same semantics as `c:GenServer.terminate/2`.
  """
  @callback terminate(reason :: term(), state :: term()) :: term()

  @doc """
  Returns the options schema for this actuator.

  The schema should NOT include the `:bb` option - it is auto-injected.
  If this callback is not implemented, the module cannot accept options
  in the DSL (must be used as a bare module).
  """
  @callback options_schema() :: Spark.Options.t()

  @doc """
  The command payloads this actuator accepts.

  Defaults to `default_command_payloads/0` — the six built-in
  `BB.Message.Actuator.Command.*` types — which is right for almost every
  driver. Override it to either end of the range:

  - **Widen it.** A driver whose hardware speaks a command BB doesn't model can
    name its own payload module here, and it will arrive through the same gated
    pipeline as any other command. Without that it would have to subscribe to
    `[:actuator | path]` itself, and those messages reach the driver having
    skipped the arm check.
  - **Narrow it.** A port that only ever accepts `Command.Effort` can say so,
    and the framework refuses everything else before the driver sees it.

  Called once at `init`, with the resolved options, because the answer isn't
  always known at compile time — it may depend on a port or channel named in
  the driver's own options.

      @impl BB.Actuator
      def command_payloads(opts) do
        [opts |> Keyword.fetch!(:port) |> MyDriver.command_struct()]
      end

  The result is used for both the actuator's pubsub subscription and its
  dispatch guard, so narrowing holds across all three transports rather than
  only the published one.

  Nothing is admitted outside this list, `Stop` included. A driver is never
  handed a payload it didn't declare, so it can't be crashed by one it has no
  clause for.
  """
  @callback command_payloads(opts :: keyword()) :: [module()]

  @typedoc """
  Something an actuator can do beyond taking commands.

  Each value names a field of `BB.Message.Sensor.JointState` the driver can
  fill in for itself, having read it back from the hardware.
  """
  @type capability :: :position_feedback | :velocity_feedback | :effort_feedback

  @doc """
  What this actuator can do besides move.

  Defaults to `[]` - the honest answer for a driver that only writes to its
  hardware, like a PWM servo or a step/direction driver. Such a joint needs a
  sensor to say where it ended up, and `BB.Dsl` warns at compile time when it
  doesn't have one.

  A driver that reads state back from the hardware - a smart servo answering
  position queries on its bus - says so here, and publishes what it reads as
  `BB.Message.Sensor.JointState` on its joint's sensor topic:

      @impl BB.Actuator
      def capabilities(_opts), do: [:position_feedback, :velocity_feedback]

  Declaring `:position_feedback` tells the framework this actuator is its own
  position sensor, so no warning is issued for the joint it drives. Declare it
  only if the driver really does publish `JointState`: the warning exists
  because `BB.Robot.State` is written from those messages and from nothing
  else, so a joint nobody reports on never moves as far as the rest of the
  framework is concerned.

  ## Options

  `opts` lets a driver answer for how it was wired up, rather than for the
  hardware in general - an encoder input that may or may not be connected:

      @impl BB.Actuator
      def capabilities(opts) do
        if opts[:feedback_pin], do: [:position_feedback], else: []
      end

  > #### These are not the options `init/1` receives {: .warning}
  >
  > This is asked at compile time, by a DSL verifier, so `opts` is what the
  > robot's author wrote in the DSL, checked against `c:options_schema/0` and
  > with its defaults applied. Two things follow:
  >
  > - There is no `:bb` key, and no `:motor_profile`. The robot doesn't exist
  >   yet.
  > - A `param()` reference can't be resolved before the robot is running, so a
  >   parameterised option arrives holding its schema default instead of the
  >   value the robot will run with. A capability that genuinely depends on one
  >   can't be answered here; say what is true of the common case, and prefer
  >   claiming a capability you sometimes lack over disclaiming one you usually
  >   have - a spurious warning teaches people to ignore warnings.
  >
  > Keep it pure for the same reason: no hardware, no processes, no `Mix`.
  """
  @callback capabilities(opts :: keyword()) :: [capability()]

  @optional_callbacks [
    capabilities: 1,
    command_payloads: 1,
    handle_options: 2,
    handle_call: 3,
    handle_cast: 2,
    handle_info: 2,
    handle_continue: 2,
    terminate: 2
  ]

  @default_command_payloads [
    BB.Message.Actuator.Command.Effort,
    BB.Message.Actuator.Command.Hold,
    BB.Message.Actuator.Command.Position,
    BB.Message.Actuator.Command.Stop,
    BB.Message.Actuator.Command.Trajectory,
    BB.Message.Actuator.Command.Velocity
  ]

  @doc """
  The command payloads an actuator accepts unless it says otherwise.

  These are the payload types `BB.Actuator`'s own API can produce, so they are
  the set every driver is expected to understand — or at least to ignore
  gracefully.
  """
  @spec default_command_payloads() :: [module()]
  def default_command_payloads, do: @default_command_payloads

  alias BB.Component.OptionsSchema

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:options_schema]

    quote do
      @behaviour BB.Actuator

      # Default implementations - all overridable
      @impl BB.Actuator
      def capabilities(_opts), do: []

      @impl BB.Actuator
      def command_payloads(_opts), do: BB.Actuator.default_command_payloads()

      @impl BB.Actuator
      def handle_options(_new_opts, state), do: {:ok, state}

      @impl BB.Actuator
      def handle_call(_request, _from, state), do: {:reply, {:error, :not_implemented}, state}

      @impl BB.Actuator
      def handle_cast(_request, state), do: {:noreply, state}

      @impl BB.Actuator
      def handle_info(_msg, state), do: {:noreply, state}

      @impl BB.Actuator
      def handle_continue(_continue_arg, state), do: {:noreply, state}

      @impl BB.Actuator
      def terminate(_reason, _state), do: :ok

      defoverridable capabilities: 1,
                     command_payloads: 1,
                     handle_options: 2,
                     handle_call: 3,
                     handle_cast: 2,
                     handle_info: 2,
                     handle_continue: 2,
                     terminate: 2

      unquote(OptionsSchema.inject(BB.Actuator, schema_opts))
    end
  end

  # ----------------------------------------------------------------------------
  # API
  # ----------------------------------------------------------------------------

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Robot
  alias BB.Safety
  alias BB.Transmission
  alias BB.Transmission.Resolver, as: TransmissionResolver
  alias Spark.Options.ValidationError

  @typedoc """
  How to address an actuator: its unique name, or its full path through the
  topology. Every function below accepts either.
  """
  @type target :: atom() | [atom()]

  @typedoc """
  How to get a command to an actuator.

  `:pubsub` publishes and then calls, `:broadcast` only publishes, and
  `:direct` only casts. See the "Delivery Methods" section above.
  """
  @type delivery :: :pubsub | :broadcast | :direct

  # ----------------------------------------------------------------------------
  # Outbound publishing
  # ----------------------------------------------------------------------------

  @doc """
  Translate a motor-space outbound message into joint-space using the
  transmission of the joint above the actuator at `actuator_path`.

  Convenient for callers that build a message in motor-space and then
  publish it on a topic of their own choosing — e.g. a controller
  publishing `JointState` on a sensor topic. Performs a fresh
  transmission resolution against the current parameter store on every
  call, so it stays correct across runtime parameter changes without
  the caller needing to subscribe.

  Returns the message unchanged when the joint has no transmission.
  """
  @spec to_joint_space(module(), [atom()], Message.t()) :: Message.t()
  def to_joint_space(robot, actuator_path, %Message{} = motor_message) do
    actuator_name = List.last(actuator_path)
    transmission = TransmissionResolver.resolve(robot, :actuator, actuator_name)
    Transmission.unapply_to_payload(motor_message, transmission)
  end

  @doc """
  Publish a `BeginMotion` message for the actuator at `path`, converting
  the supplied motor-space values into joint-space before publishing.

  The driver builds the message in motor-space (the only coordinate space
  it knows about); this helper looks up the joint above the actuator,
  resolves its transmission against the current parameter store, applies
  `BB.Transmission.unapply_to_payload/2`, and publishes the joint-space
  message to `[:actuator | path]`.

  `path` is the actuator's full path (i.e. the `:bb.path` injected into
  driver opts). `opts` is the keyword list accepted by
  `BB.Message.Actuator.BeginMotion`'s schema, with `:initial_position`,
  `:target_position`, `:peak_velocity`, and `:acceleration` in
  motor-space.
  """
  @spec publish_begin_motion(module(), [atom()], keyword()) :: :ok
  def publish_begin_motion(robot, path, opts) do
    joint_name = joint_name_for_actuator(robot, path)
    {:ok, motor_message} = Message.new(BeginMotion, joint_name, opts)
    joint_message = to_joint_space(robot, path, motor_message)
    BB.publish(robot, [:actuator | path], joint_message)
  end

  defp joint_name_for_actuator(robot, path) do
    actuator_name = List.last(path)

    case Map.get(robot.robot().actuators, actuator_name) do
      %{joint: joint_name} -> joint_name
      _ -> actuator_name
    end
  end

  # ----------------------------------------------------------------------------
  # Command options
  # ----------------------------------------------------------------------------

  @default_timeout 5000

  @delivery_values [:pubsub, :broadcast, :direct]

  @delivery_doc """
  How to get the command to the actuator. `:pubsub` publishes it and waits for \
  the actuator to accept or refuse it; `:broadcast` only publishes it; \
  `:direct` only casts it. See the "Delivery Methods" section of `BB.Actuator`.\
  """

  @reply_on_reject_doc """
  Have the actuator send `{:bb, :command_rejected, actuator_name, command_id, \
  error}` to the calling process when it refuses the command, rather than \
  refusing in silence. Valid only with `delivery: :direct`.\
  """

  # Spelled out per command rather than shared, because the default is the one
  # thing about delivery that differs between them.
  @delivery_opt [
    type: {:in, @delivery_values},
    doc: @delivery_doc
  ]

  @duration_opt [
    type: {:or, [nil, :pos_integer]},
    doc: "Duration hint (milliseconds), `nil` meaning until countermanded."
  ]

  @common_section "Common to every actuator command"

  # Shared by every command, so that a caller who learns one learns all six.
  @common_command_opts [
    reply_on_reject?: [
      type: :boolean,
      default: false,
      doc: @reply_on_reject_doc
    ],
    command_id: [
      type: {:or, [nil, :reference]},
      doc: "Correlation ID for feedback tracking, and for the `:reply_on_reject?` reply."
    ],
    timeout: [
      type: :timeout,
      default: @default_timeout,
      doc:
        "How long to wait for the actuator, in milliseconds. Used only under `delivery: :pubsub`."
    ]
  ]

  @position_schema Spark.Options.new!(
                     Spark.Options.merge(
                       [
                         delivery: Keyword.put(@delivery_opt, :default, :pubsub),
                         velocity: [
                           type: {:or, [nil, :float]},
                           doc: "Velocity hint (rad/s or m/s)."
                         ],
                         duration: @duration_opt
                       ],
                       @common_command_opts,
                       @common_section
                     )
                   )

  @velocity_schema Spark.Options.new!(
                     Spark.Options.merge(
                       [
                         delivery: Keyword.put(@delivery_opt, :default, :broadcast),
                         duration: @duration_opt
                       ],
                       @common_command_opts,
                       @common_section
                     )
                   )

  @effort_schema Spark.Options.new!(
                   Spark.Options.merge(
                     [
                       delivery: Keyword.put(@delivery_opt, :default, :broadcast),
                       duration: @duration_opt
                     ],
                     @common_command_opts,
                     @common_section
                   )
                 )

  @trajectory_schema Spark.Options.new!(
                       Spark.Options.merge(
                         [
                           delivery: Keyword.put(@delivery_opt, :default, :broadcast),
                           repeat: [
                             type: {:or, [:pos_integer, {:in, [:forever]}]},
                             default: 1,
                             doc: "Number of repetitions: a positive integer, or `:forever`."
                           ]
                         ],
                         @common_command_opts,
                         @common_section
                       )
                     )

  @stop_schema Spark.Options.new!(
                 Spark.Options.merge(
                   [
                     delivery: Keyword.put(@delivery_opt, :default, :broadcast),
                     mode: [
                       type: {:in, [:immediate, :decelerate]},
                       default: :immediate,
                       doc: "How to come to a stop."
                     ]
                   ],
                   @common_command_opts,
                   @common_section
                 )
               )

  @hold_schema Spark.Options.new!(
                 Spark.Options.merge(
                   [delivery: Keyword.put(@delivery_opt, :default, :broadcast)],
                   @common_command_opts,
                   @common_section
                 )
               )

  # ----------------------------------------------------------------------------
  # Position Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a position command.

  Under the default `delivery: :pubsub` the command is published to
  `[:actuator | path]` for whoever is watching the topic, and delivered to the
  actuator itself by a call, so the caller learns whether the joint is actually
  moving. An actuator refuses a command it doesn't accept, or any command at
  all while the robot is disarmed.

  The publication records that a command was issued; the return value says
  whether it was accepted. A refused command still appears on the topic.

  Runs in the caller's process, so a driver must not call this from inside its
  own `c:handle_command/2` - it would be waiting on itself.

  ## Options

  #{Spark.Options.docs(@position_schema)}

  ## Returns

  - `:ok` - Under `:pubsub`, that the actuator accepted the command. Under
    `:broadcast` and `:direct`, only that it was sent
  - `{:error, reason}` - `:pubsub` only. The actuator refused, `reason` being a
    `BB.Error` struct

  Under `delivery: :pubsub`, exits if the actuator isn't running or doesn't
  answer within `:timeout`, like any other `GenServer.call/3`.

  > #### `:broadcast` and `:direct` cannot report a refusal {: .warning}
  >
  > Neither waits for the actuator, so both return `:ok` whether the command
  > was accepted or thrown out. The refusal reaches the log and a
  > `[:bb, :actuator, :rejected]` telemetry event, and nowhere else — matching
  > on `{:error, reason}` there is a branch that can never run.
  >
  > This is how a disarmed robot, an unsupported payload or a stale arm epoch
  > goes unnoticed. Under `:direct`, `reply_on_reject?: true` has the actuator
  > message the caller instead of staying silent; `:broadcast` has no such
  > remedy, because a published command has no one caller to answer.
  >

  ## Commanding several joints

  This is an ordinary blocking call, so several joints cost several round
  trips and how they overlap is yours to choose:

      [shoulder: 1.57, elbow: 0.5]
      |> Enum.map(fn {joint, position} ->
        Task.async(fn -> BB.Actuator.set_position(MyRobot, joint, position) end)
      end)
      |> Task.await_many()

  Three things to know before reaching for that:

  - `Task.async/1` **links**. From a long-lived process - a controller loop -
    use `Task.Supervisor.async_nolink/3` instead, or an actuator that has died
    takes the caller down with it.
  - `Task.await_many/2` applies one deadline to the whole set and kills the
    stragglers when it expires. It isn't "collect as they land".
  - Each command publishes from inside its own task, so observers see the
    commands interleaved rather than in the order you listed them.

  `BB.Motion.send_positions/3` already does this for the joints of one motion.

  ## Examples

      :ok = BB.Actuator.set_position(MyRobot, [:base_link, :shoulder, :servo], 1.57)

      case BB.Actuator.set_position(MyRobot, :servo, 1.57, velocity: 0.5) do
        :ok -> :moving
        {:error, error} -> Logger.error(Exception.message(error))
      end
  """
  @spec set_position(module(), target(), number(), keyword()) :: :ok | {:error, term()}
  def set_position(robot, target, position, opts \\ []) do
    opts = validate_opts!(opts, @position_schema)

    deliver(robot, target, opts, fn actuator_name ->
      Message.new!(Command.Position, actuator_name,
        position: position * 1.0,
        velocity: opts[:velocity],
        duration: opts[:duration],
        command_id: opts[:command_id]
      )
    end)
  end

  # ----------------------------------------------------------------------------
  # Velocity Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a velocity command.

  Defaults to `delivery: :broadcast`: the command is published to
  `[:actuator | path]`, where the actuator and every other subscriber picks it
  up, and the caller doesn't wait. Velocity commands sit on control loops, so
  the default keeps them non-blocking; pass `delivery: :pubsub` to be told
  whether the actuator took it.

  ## Options

  #{Spark.Options.docs(@velocity_schema)}

  ## Returns

  - `:ok` - Under `:pubsub`, that the actuator accepted the command. Under the
    default `:broadcast`, and under `:direct`, only that it was sent
  - `{:error, reason}` - `:pubsub` only. The actuator refused, `reason` being a
    `BB.Error` struct

  See `set_position/4` for what `:broadcast` and `:direct` cannot tell you, and
  what `:reply_on_reject?` does about it.

  ## Examples

      # Doesn't wait, and cannot report a refusal
      :ok = BB.Actuator.set_velocity(MyRobot, :wheel, 2.5)

      # Waits for the actuator
      case BB.Actuator.set_velocity(MyRobot, :wheel, 2.5, delivery: :pubsub) do
        :ok -> :turning
        {:error, error} -> Logger.error(Exception.message(error))
      end
  """
  @spec set_velocity(module(), target(), number(), keyword()) :: :ok | {:error, term()}
  def set_velocity(robot, target, velocity, opts \\ []) do
    opts = validate_opts!(opts, @velocity_schema)

    deliver(robot, target, opts, fn actuator_name ->
      Message.new!(Command.Velocity, actuator_name,
        velocity: velocity * 1.0,
        duration: opts[:duration],
        command_id: opts[:command_id]
      )
    end)
  end

  # ----------------------------------------------------------------------------
  # Effort Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send an effort (torque/force) command.

  Defaults to `delivery: :broadcast`: the command is published to
  `[:actuator | path]`, where the actuator and every other subscriber picks it
  up, and the caller doesn't wait. Effort commands sit on control loops, so the
  default keeps them non-blocking; pass `delivery: :pubsub` to be told whether
  the actuator took it.

  ## Options

  #{Spark.Options.docs(@effort_schema)}

  ## Returns

  - `:ok` - Under `:pubsub`, that the actuator accepted the command. Under the
    default `:broadcast`, and under `:direct`, only that it was sent
  - `{:error, reason}` - `:pubsub` only. The actuator refused, `reason` being a
    `BB.Error` struct

  See `set_position/4` for what `:broadcast` and `:direct` cannot tell you, and
  what `:reply_on_reject?` does about it.
  """
  @spec set_effort(module(), target(), number(), keyword()) :: :ok | {:error, term()}
  def set_effort(robot, target, effort, opts \\ []) do
    opts = validate_opts!(opts, @effort_schema)

    deliver(robot, target, opts, fn actuator_name ->
      Message.new!(Command.Effort, actuator_name,
        effort: effort * 1.0,
        duration: opts[:duration],
        command_id: opts[:command_id]
      )
    end)
  end

  # ----------------------------------------------------------------------------
  # Trajectory Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a trajectory command.

  Defaults to `delivery: :broadcast`: the command is published to
  `[:actuator | path]`, where the actuator and every other subscriber picks it
  up, and the caller doesn't wait. Pass `delivery: :pubsub` to be told whether
  the actuator took it.

  ## Waypoint Structure

  Each waypoint should be a keyword list or map with:
  - `position` - Position (radians or metres)
  - `velocity` - Velocity (rad/s or m/s)
  - `acceleration` - Acceleration (rad/s² or m/s²)
  - `time_from_start` - Time from trajectory start (milliseconds)

  ## Options

  #{Spark.Options.docs(@trajectory_schema)}

  ## Returns

  - `:ok` - Under `:pubsub`, that the actuator accepted the trajectory. Under
    the default `:broadcast`, and under `:direct`, only that it was sent
  - `{:error, reason}` - `:pubsub` only. The actuator refused, `reason` being a
    `BB.Error` struct

  A driver that doesn't declare `BB.Message.Actuator.Command.Trajectory` in
  `c:command_payloads/1` refuses this, which is precisely the refusal the
  default delivery cannot report. See `set_position/4` for what
  `:reply_on_reject?` does about it.
  """
  @spec follow_trajectory(module(), target(), [keyword() | map()], keyword()) ::
          :ok | {:error, term()}
  def follow_trajectory(robot, target, waypoints, opts \\ []) do
    opts = validate_opts!(opts, @trajectory_schema)
    normalised_waypoints = Enum.map(waypoints, &normalise_waypoint/1)

    deliver(robot, target, opts, fn actuator_name ->
      Message.new!(Command.Trajectory, actuator_name,
        waypoints: normalised_waypoints,
        repeat: opts[:repeat],
        command_id: opts[:command_id]
      )
    end)
  end

  defp as_float(nil), do: nil
  defp as_float(value), do: value * 1.0

  defp normalise_waypoint(waypoint) when is_map(waypoint),
    do: waypoint |> Keyword.new() |> normalise_waypoint()

  defp normalise_waypoint(waypoint) when is_list(waypoint) do
    [
      position: waypoint[:position] * 1.0,
      velocity: as_float(waypoint[:velocity]),
      acceleration: as_float(waypoint[:acceleration]),
      time_from_start: waypoint[:time_from_start]
    ]
  end

  # ----------------------------------------------------------------------------
  # Stop Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a stop command.

  Tells the actuator to cease travelling and become passive. This is a motion
  command, not a safety one - making hardware safe is `BB.Safety.disarm/1`.

  Defaults to `delivery: :broadcast`: the command is published to
  `[:actuator | path]`, where the actuator and every other subscriber picks it
  up, and the caller doesn't wait. Pass `delivery: :pubsub` to be told whether
  the actuator took it.

  ## Options

  #{Spark.Options.docs(@stop_schema)}

  ## Returns

  - `:ok` - Under `:pubsub`, that the actuator accepted the command. Under the
    default `:broadcast`, and under `:direct`, only that it was sent
  - `{:error, reason}` - `:pubsub` only. The actuator refused, `reason` being a
    `BB.Error` struct

  > #### A stop you didn't wait for is a stop you can't confirm {: .warning}
  >
  > Under the default `:broadcast` this returns `:ok` even when the robot is
  > disarmed, the arm epoch is stale, or the driver never declared
  > `Command.Stop` - none of which stop the joint. Use `delivery: :pubsub` when
  > the stop has to be confirmed, or `delivery: :direct` with
  > `reply_on_reject?: true` when it has to be prompt and you still want to
  > hear about a refusal.
  >
  """
  @spec stop(module(), target(), keyword()) :: :ok | {:error, term()}
  def stop(robot, target, opts \\ []) do
    opts = validate_opts!(opts, @stop_schema)

    deliver(robot, target, opts, fn actuator_name ->
      Message.new!(Command.Stop, actuator_name,
        mode: opts[:mode],
        command_id: opts[:command_id]
      )
    end)
  end

  # ----------------------------------------------------------------------------
  # Hold Commands
  # ----------------------------------------------------------------------------

  @doc """
  Send a hold command.

  Tells the actuator to actively maintain its current position, resisting
  external force - the counterpart to `stop/3`, which goes passive.

  Defaults to `delivery: :broadcast`: the command is published to
  `[:actuator | path]`, where the actuator and every other subscriber picks it
  up, and the caller doesn't wait. Pass `delivery: :pubsub` to be told whether
  the actuator took it.

  ## Options

  #{Spark.Options.docs(@hold_schema)}

  ## Returns

  - `:ok` - Under `:pubsub`, that the actuator accepted the command. Under the
    default `:broadcast`, and under `:direct`, only that it was sent
  - `{:error, reason}` - `:pubsub` only. The actuator refused, `reason` being a
    `BB.Error` struct

  See `set_position/4` for what `:broadcast` and `:direct` cannot tell you, and
  what `:reply_on_reject?` does about it.
  """
  @spec hold(module(), target(), keyword()) :: :ok | {:error, term()}
  def hold(robot, target, opts \\ []) do
    opts = validate_opts!(opts, @hold_schema)

    deliver(robot, target, opts, fn actuator_name ->
      Message.new!(Command.Hold, actuator_name, command_id: opts[:command_id])
    end)
  end

  # ----------------------------------------------------------------------------
  # Delivery
  # ----------------------------------------------------------------------------

  # `build` is handed the actuator's name to use as the message's frame id, and
  # is called only once the target has resolved, so a bad target raises before
  # a message is built.
  @spec deliver(module(), target(), keyword(), (atom() -> Message.t())) ::
          :ok | {:error, term()}
  defp deliver(robot, target, opts, build),
    do: deliver(Keyword.fetch!(opts, :delivery), robot, target, opts, build)

  # The actuator is excluded from the publication because it is about to be
  # handed the same command directly: it subscribes to its own command topic,
  # and would otherwise drive the hardware twice for one call.
  defp deliver(:pubsub, robot, target, opts, build) do
    path = actuator_path!(robot, target)
    actuator_name = List.last(path)
    message = stamp(robot, build.(actuator_name))

    BB.publish(robot, [:actuator | path], message,
      except: [BB.Process.whereis(robot, actuator_name)]
    )

    robot
    |> BB.call(actuator_name, {:command, message}, Keyword.fetch!(opts, :timeout))
    |> command_result()
  end

  defp deliver(:broadcast, robot, target, _opts, build) do
    path = actuator_path!(robot, target)
    message = stamp(robot, build.(List.last(path)))
    BB.publish(robot, [:actuator | path], message)
  end

  defp deliver(:direct, robot, target, opts, build) do
    actuator_name = actuator_name!(robot, target)
    message = stamp(robot, build.(actuator_name))
    BB.cast(robot, actuator_name, {:command, message, reply_target(opts)})
  end

  defp reply_target(opts) do
    if Keyword.fetch!(opts, :reply_on_reject?), do: self()
  end

  @spec validate_opts!(keyword(), Spark.Options.t()) :: keyword()
  defp validate_opts!(opts, schema) do
    opts
    |> Spark.Options.validate!(schema)
    |> validate_reply_delivery!()
  end

  # `reply_on_reject?` needs one caller to answer and a tuple to carry its pid,
  # and only `:direct` has both. Spark validates each key on its own, so this
  # cross-key rule is checked here — as a `Spark.Options.ValidationError`, so a
  # caller sees one kind of option failure rather than two.
  defp validate_reply_delivery!(opts),
    do:
      validate_reply_delivery!(
        opts,
        Keyword.fetch!(opts, :delivery),
        Keyword.fetch!(opts, :reply_on_reject?)
      )

  defp validate_reply_delivery!(opts, :direct, _reply_on_reject?), do: opts
  defp validate_reply_delivery!(opts, _delivery, false), do: opts

  defp validate_reply_delivery!(_opts, :pubsub, true) do
    raise ValidationError.exception(
            key: :reply_on_reject?,
            value: true,
            message:
              "invalid value for :reply_on_reject? option: cannot be combined with " <>
                "`delivery: :pubsub`, which waits for the actuator and returns " <>
                "`{:error, reason}` on a refusal already. Drop `:reply_on_reject?`, or pass " <>
                "`delivery: :direct` to hear about refusals without waiting."
          )
  end

  defp validate_reply_delivery!(_opts, :broadcast, true) do
    raise ValidationError.exception(
            key: :reply_on_reject?,
            value: true,
            message:
              "invalid value for :reply_on_reject? option: cannot be combined with " <>
                "`delivery: :broadcast`, which publishes to every subscriber and so has no " <>
                "one caller to reply to. Pass `delivery: :direct` to hear about refusals " <>
                "without waiting."
          )
  end

  # Stamped here rather than in `BB.Message.new/3` because only a send is an
  # attempt to drive hardware: a message may be built long before, or while the
  # robot is disarmed, and what authorises it is the epoch in force as it goes
  # out. An unarmed robot leaves the stamp empty, and the command is refused on
  # arrival either way.
  @spec stamp(module(), Message.t()) :: Message.t()
  defp stamp(robot, message) do
    case Safety.epoch(robot) do
      {:ok, epoch} -> %{message | arm_epoch: epoch}
      :error -> message
    end
  end

  # A driver may answer `c:handle_command/2` with anything it likes; only a
  # refusal changes what the caller should do next.
  @spec command_result(term()) :: :ok | {:error, term()}
  defp command_result({:error, reason}), do: {:error, reason}
  defp command_result(_accepted), do: :ok

  # ----------------------------------------------------------------------------
  # Addressing
  # ----------------------------------------------------------------------------

  # Pubsub delivery needs the actuator's full path, because that is the topic its
  # server subscribes to. A bare name is resolved against the robot rather than
  # passed through: `[:actuator | :servo]` is an improper list, which slips past
  # `BB.publish/3`'s `is_list` guard and then dies inside `Enum.scan/3`.
  @spec actuator_path!(module(), target()) :: [atom()]
  defp actuator_path!(_robot, path) when is_list(path), do: path

  defp actuator_path!(robot, name) when is_atom(name) do
    case Robot.actuator_path(robot.robot(), name) do
      {:ok, path} ->
        path

      {:error, _} ->
        raise ArgumentError,
              "#{inspect(robot)} has no actuator named #{inspect(name)}. " <>
                "Known actuators: #{inspect(Map.keys(robot.robot().actuators))}"
    end
  end

  # Direct and synchronous delivery go through the registry, which is keyed by
  # the actuator's unique name.
  @spec actuator_name!(module(), target()) :: atom()
  defp actuator_name!(_robot, name) when is_atom(name), do: name
  defp actuator_name!(_robot, path) when is_list(path), do: List.last(path)
end
