# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.Server do
  @moduledoc """
  Wrapper GenServer for actuator callback modules.

  This module manages the lifecycle of user-defined actuator modules, handling:
  - Parameter reference resolution at startup
  - Subscription to parameter changes
  - Delegation of GenServer callbacks to user module
  - Automatic safety registration

  User modules implement the `BB.Actuator` behaviour and define callbacks.
  This server wraps them, providing the actual GenServer implementation.
  """

  use GenServer

  alias BB.Actuator.MotorProfile
  alias BB.Component.OptionsSchema
  alias BB.Message
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot
  alias BB.Server.ParamResolution
  alias BB.Transmission
  alias BB.Transmission.Resolver, as: TransmissionResolver

  @framework_keys [:bb, :motor_profile]

  defstruct [
    :callback_module,
    :resolved_opts,
    :raw_opts,
    :param_subscriptions,
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
          transmission: Transmission.t() | nil,
          transmission_subscriptions: %{atom() => [atom()]},
          joint: map() | nil,
          joint_name: atom() | nil,
          actuator_name: atom() | nil,
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

    {param_subscriptions, resolved_opts} =
      ParamResolution.resolve_and_subscribe(raw_opts, bb.robot)

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
        base = %__MODULE__{
          callback_module: callback_module,
          resolved_opts: resolved_opts,
          raw_opts: raw_opts,
          param_subscriptions: param_subscriptions,
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

  def handle_info({:bb, topic, %Message{} = message}, state) do
    transformed = Transmission.apply_to_command(message, state.transmission)
    delegate_handle_info({:bb, topic, transformed}, state)
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
  def handle_call({:command, %Message{} = message}, from, state) do
    transformed = Transmission.apply_to_command(message, state.transmission)
    delegate_handle_call({:command, transformed}, from, state)
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
    transformed = Transmission.apply_to_command(message, state.transmission)
    delegate_handle_cast({:command, transformed}, state)
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
