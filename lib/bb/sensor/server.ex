# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.Server do
  @moduledoc """
  Wrapper GenServer for sensor callback modules.

  This module manages the lifecycle of user-defined sensor modules, handling:
  - Parameter reference resolution at startup
  - Subscription to parameter changes
  - For joint-attached sensors, transmission resolution and a `:sensor_profile` opt
  - Delegation of GenServer callbacks to user module
  - Optional safety registration (only if sensor implements `disarm/1`)

  User modules implement the `BB.Sensor` behaviour and define callbacks.
  This server wraps them, providing the actual GenServer implementation.
  """

  use GenServer

  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Sensor.SensorProfile
  alias BB.Server.ParamResolution
  alias BB.Transmission
  alias BB.Transmission.Resolver, as: TransmissionResolver

  defstruct [
    :callback_module,
    :resolved_opts,
    :raw_opts,
    :param_subscriptions,
    :transmission,
    :transmission_subscriptions,
    :sensor_name,
    :joint_name,
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
          sensor_name: atom() | nil,
          joint_name: atom() | nil,
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

    sensor_name = List.last(bb.path)
    joint_name = joint_name_for_sensor(bb, sensor_name)

    {transmission, transmission_subscriptions} =
      if joint_name do
        TransmissionResolver.resolve_and_subscribe(bb.robot, :sensor, sensor_name)
      else
        {nil, %{}}
      end

    sensor_profile = %SensorProfile{joint_name: joint_name, transmission: transmission}
    resolved_opts = Keyword.put(resolved_opts, :sensor_profile, sensor_profile)

    base = %__MODULE__{
      callback_module: callback_module,
      resolved_opts: resolved_opts,
      raw_opts: raw_opts,
      param_subscriptions: param_subscriptions,
      transmission: transmission,
      transmission_subscriptions: transmission_subscriptions,
      sensor_name: sensor_name,
      joint_name: joint_name,
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

  defp joint_name_for_sensor(%{robot: robot_module}, sensor_name) do
    case Map.get(robot_module.robot().sensors, sensor_name) do
      %{attached_to: {:joint, joint_name}} -> joint_name
      _ -> nil
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
             :sensor,
             state.sensor_name
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
          :ignored -> Keyword.delete(state.resolved_opts, :sensor_profile)
        end

      new_resolved =
        Keyword.put(
          base_opts,
          :sensor_profile,
          %SensorProfile{joint_name: state.joint_name, transmission: state.transmission}
        )

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
end
