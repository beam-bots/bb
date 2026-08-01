# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.Server do
  @moduledoc """
  Wrapper GenServer for actuator callback modules.

  This module manages the lifecycle of user-defined actuator modules, handling:
  - Parameter reference resolution at startup
  - Subscription to parameter changes
  - Subscription to the actuator's own command topic
  - Delegation of GenServer callbacks to user module
  - Automatic safety registration

  User modules implement the `BB.Actuator` behaviour and define callbacks.
  This server wraps them, providing the actual GenServer implementation.

  ## The inbound command pipeline

  Commands reach an actuator by three transports - published to
  `[:actuator | path]`, cast via `BB.Process.cast/3`, or called via
  `BB.Process.call/4`. All three converge here and pass through the same
  checks before the driver sees them:

  1. The actuator must accept the payload — see
     `c:BB.Actuator.command_payloads/1`. A driver is never handed a command it
     didn't declare, so it can't be crashed by one it has no clause for.
  2. The robot must be armed.
  3. The payload is translated from joint-space into motor-space using the
     joint's transmission.

  The driver then receives it in `c:BB.Actuator.handle_command/2`, and its
  reply is routed back to whichever transport delivered the command. Messages
  from topics the driver subscribed to itself are not part of this pipeline -
  they reach `c:BB.Actuator.handle_info/2` untouched.
  """

  use GenServer

  alias BB.Actuator.MotorProfile
  alias BB.Component.OptionsSchema
  alias BB.Error.State.NotArmed
  alias BB.Error.State.UnsupportedCommand
  alias BB.Message
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot
  alias BB.Safety
  alias BB.Server.ParamResolution
  alias BB.Transmission
  alias BB.Transmission.Resolver, as: TransmissionResolver

  @framework_keys [:bb, :motor_profile]

  defstruct [
    :callback_module,
    :resolved_opts,
    :raw_opts,
    :param_subscriptions,
    :command_topic,
    :command_payloads,
    :transmission,
    :transmission_subscriptions,
    :joint,
    :joint_name,
    :actuator_name,
    :bb,
    :user_state
  ]

  @type t :: %__MODULE__{
          callback_module: module(),
          resolved_opts: keyword(),
          raw_opts: keyword(),
          param_subscriptions: %{[atom()] => atom()},
          command_topic: [atom()],
          command_payloads: [module()],
          transmission: Transmission.t() | nil,
          transmission_subscriptions: %{atom() => [atom()]},
          joint: map() | nil,
          joint_name: atom() | nil,
          actuator_name: atom() | nil,
          bb: %{robot: module(), path: [atom()]},
          user_state: term()
        }

  @typep transport :: :call | :cast | :pubsub

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

    {param_subscriptions, resolved_opts} =
      ParamResolution.resolve_and_subscribe(raw_opts, bb.robot)

    command_topic = [:actuator | bb.path]

    actuator_name = List.last(bb.path)
    {joint, joint_name} = joint_for_actuator(bb)

    {transmission, transmission_subscriptions} =
      if actuator_name do
        TransmissionResolver.resolve_and_subscribe(bb.robot, :actuator, actuator_name)
      else
        {nil, %{}}
      end

    motor_profile = MotorProfile.from_joint(joint, transmission)
    resolved_opts = Keyword.put(resolved_opts, :motor_profile, motor_profile)

    case OptionsSchema.validate(callback_module, resolved_opts, @framework_keys) do
      {:error, error} ->
        {:stop, error}

      {:ok, resolved_opts} ->
        # Asked after validation, because a driver may derive its accepted
        # payloads from its own options. The same list gates every transport,
        # so narrowing holds for cast and call too, not just the published path.
        #
        command_payloads = callback_module.command_payloads(resolved_opts)

        BB.PubSub.subscribe(bb.robot, command_topic, message_types: command_payloads)

        base = %__MODULE__{
          callback_module: callback_module,
          resolved_opts: resolved_opts,
          raw_opts: raw_opts,
          param_subscriptions: param_subscriptions,
          command_topic: command_topic,
          command_payloads: command_payloads,
          transmission: transmission,
          transmission_subscriptions: transmission_subscriptions,
          joint: joint,
          joint_name: joint_name,
          actuator_name: actuator_name,
          bb: bb
        }

        case callback_module.init(resolved_opts) do
          {:ok, user_state} ->
            {:ok, %{base | user_state: user_state}}

          {:ok, user_state, timeout_or_continue} ->
            {:ok, %{base | user_state: user_state}, timeout_or_continue}

          {:stop, reason} ->
            {:stop, reason}

          :ignore ->
            :ignore
        end
    end
  end

  defp joint_for_actuator(%{robot: robot_module, path: path}) do
    actuator_name = List.last(path)
    robot = robot_module.robot()

    case Map.get(robot.actuators, actuator_name) do
      %{joint: joint_name} -> {Robot.get_joint(robot, joint_name), joint_name}
      _ -> {nil, nil}
    end
  end

  @impl GenServer
  def handle_info({:bb, [:param | param_path], %{payload: %ParameterChanged{}}}, state) do
    {transmission_changed?, state} =
      case TransmissionResolver.handle_change(
             param_path,
             state.transmission,
             state.transmission_subscriptions,
             state.bb.robot,
             :actuator,
             state.actuator_name
           ) do
        {:changed, new_transmission} ->
          {true, %{state | transmission: new_transmission}}

        :ignored ->
          {false, state}
      end

    param_result =
      ParamResolution.handle_change(
        param_path,
        state.param_subscriptions,
        state.raw_opts,
        state.bb.robot
      )

    if transmission_changed? or match?({:changed, _}, param_result) do
      base_opts =
        case param_result do
          {:changed, opts} -> opts
          :ignored -> Keyword.delete(state.resolved_opts, :motor_profile)
        end

      new_resolved =
        Keyword.put(
          base_opts,
          :motor_profile,
          MotorProfile.from_joint(state.joint, state.transmission)
        )

      with {:ok, new_resolved} <-
             OptionsSchema.validate(state.callback_module, new_resolved, @framework_keys),
           {:ok, new_user_state} <-
             state.callback_module.handle_options(new_resolved, state.user_state) do
        {:noreply, %{state | resolved_opts: new_resolved, user_state: new_user_state}}
      else
        {:stop, reason} -> {:stop, reason, state}
        {:error, error} -> {:stop, error, state}
      end
    else
      {:noreply, state}
    end
  end

  def handle_info(
        {:bb, topic, %Message{} = message} = msg,
        %__MODULE__{command_topic: topic} = state
      ) do
    if command?(message, state) do
      dispatch_command(message, :pubsub, state)
    else
      delegate_handle_info(msg, state)
    end
  end

  def handle_info(msg, state), do: delegate_handle_info(msg, state)

  defp delegate_handle_info(msg, state) do
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
  def handle_call({:command, %Message{} = message}, _from, state) do
    dispatch_command(message, :call, state)
  end

  def handle_call({:command, _other}, _from, state) do
    {:reply, {:error, :not_a_command}, state}
  end

  def handle_call(request, from, state), do: delegate_handle_call(request, from, state)

  defp delegate_handle_call(request, from, state) do
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
  def handle_cast({:command, %Message{} = message}, state) do
    dispatch_command(message, :cast, state)
  end

  def handle_cast(request, state), do: delegate_handle_cast(request, state)

  defp delegate_handle_cast(request, state) do
    case state.callback_module.handle_cast(request, state.user_state) do
      {:noreply, new_user_state} ->
        {:noreply, %{state | user_state: new_user_state}}

      {:noreply, new_user_state, timeout_or_continue} ->
        {:noreply, %{state | user_state: new_user_state}, timeout_or_continue}

      {:stop, reason, new_user_state} ->
        {:stop, reason, %{state | user_state: new_user_state}}
    end
  end

  # Only payloads this actuator accepts are commands. Anything else on the topic
  # is somebody else's traffic and belongs to the driver's `handle_info/2`.
  @spec command?(Message.t(), t()) :: boolean()
  defp command?(%Message{payload: %payload_module{}}, state),
    do: payload_module in state.command_payloads

  @spec dispatch_command(Message.t(), transport(), t()) :: term()
  defp dispatch_command(message, transport, state) do
    case authorise(message, state) do
      :ok ->
        message
        |> Transmission.apply_to_command(state.transmission)
        |> delegate_handle_command(transport, state)

      {:error, reason, error} ->
        refuse(message, reason, error, transport, state)
    end
  end

  defp authorise(%Message{payload: %payload_module{}} = message, state) do
    if payload_module in state.command_payloads do
      authorise_armed(message, state)
    else
      {:error, :unsupported_command,
       UnsupportedCommand.exception(
         robot: state.bb.robot,
         actuator: state.actuator_name,
         command: payload_module,
         supported: state.command_payloads
       )}
    end
  end

  defp authorise_armed(%Message{payload: %payload_module{}}, state) do
    if Safety.armed?(state.bb.robot) do
      :ok
    else
      {:error, :disarmed,
       NotArmed.exception(
         robot: state.bb.robot,
         actuator: state.actuator_name,
         command: payload_module
       )}
    end
  end

  defp refuse(message, reason, error, transport, state) do
    :telemetry.execute(
      [:bb, :actuator, :rejected],
      %{count: 1},
      %{
        robot: state.bb.robot,
        actuator: state.actuator_name,
        transport: transport,
        payload_module: message.payload.__struct__,
        reason: reason
      }
    )

    command_reply(transport, {:error, error}, state)
  end

  defp delegate_handle_command(message, transport, state) do
    case state.callback_module.handle_command(message, state.user_state) do
      {:reply, reply, new_user_state} ->
        command_reply(transport, reply, %{state | user_state: new_user_state})

      {:reply, reply, new_user_state, timeout_or_continue} ->
        command_reply(
          transport,
          reply,
          %{state | user_state: new_user_state},
          timeout_or_continue
        )

      {:noreply, new_user_state} ->
        command_reply(transport, {:ok, :accepted}, %{state | user_state: new_user_state})

      {:noreply, new_user_state, timeout_or_continue} ->
        command_reply(
          transport,
          {:ok, :accepted},
          %{state | user_state: new_user_state},
          timeout_or_continue
        )

      {:stop, reason, new_user_state} ->
        {:stop, reason, %{state | user_state: new_user_state}}
    end
  end

  # Only the synchronous transport has anywhere to put a reply; the others are
  # fire-and-forget and the server is answering `handle_cast`/`handle_info`.
  defp command_reply(transport, reply, state, timeout_or_continue \\ nil)
  defp command_reply(:call, reply, state, nil), do: {:reply, reply, state}

  defp command_reply(:call, reply, state, timeout_or_continue),
    do: {:reply, reply, state, timeout_or_continue}

  defp command_reply(_transport, _reply, state, nil), do: {:noreply, state}

  defp command_reply(_transport, _reply, state, timeout_or_continue),
    do: {:noreply, state, timeout_or_continue}

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
end
