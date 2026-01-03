# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Server.ParamResolution do
  @moduledoc """
  Shared parameter resolution logic for wrapper servers.

  This module provides functions for resolving `ParamRef` values in options
  and subscribing to parameter change notifications. Used by:

  - `BB.Actuator.Server`
  - `BB.Sensor.Server`
  - `BB.Controller.Server`
  - `BB.Command.Server`

  ## Usage

  In your server's `init/1`:

      {param_subscriptions, resolved_opts} =
        ParamResolution.resolve_and_subscribe(raw_opts, robot_module)

  In `handle_info/2` for parameter changes:

      def handle_info({:bb, [:param | param_path], %{payload: %ParameterChanged{}}}, state) do
        ParamResolution.handle_param_change(
          param_path,
          state.param_subscriptions,
          state.raw_opts,
          robot_module,
          fn new_resolved ->
            # Call user's handle_options callback
          end
        )
      end
  """

  alias BB.Dsl.ParamRef
  alias BB.PubSub
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  @type subscriptions :: %{[atom()] => atom()}

  @doc """
  Resolve ParamRefs in options and subscribe to parameter change topics.

  Returns `{subscriptions_map, resolved_opts}` where:
  - `subscriptions_map` maps parameter paths to option keys
  - `resolved_opts` has ParamRefs replaced with their current values

  This is a convenience function that calls `resolve/2` and `subscribe/2`.
  """
  @spec resolve_and_subscribe(keyword(), module() | RobotState.t()) ::
          {subscriptions(), keyword()}
  def resolve_and_subscribe(opts, robot_module_or_state) do
    {robot_module, robot_state} = normalise_robot_arg(robot_module_or_state)
    {subscriptions, resolved} = resolve(opts, robot_state)
    subscribe(robot_module, subscriptions)
    {subscriptions, resolved}
  end

  @doc """
  Resolve ParamRefs in options without subscribing.

  Takes either a robot module or a `BB.Robot.State` struct directly.
  Passing the state directly avoids a lookup and potential deadlock
  during init when the Runtime is still starting.
  """
  @spec resolve(keyword(), module() | RobotState.t()) :: {subscriptions(), keyword()}
  def resolve(opts, robot_module_or_state) do
    robot_state =
      case robot_module_or_state do
        %RobotState{} = state -> state
        module when is_atom(module) -> Runtime.get_robot_state(module)
      end

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

  @doc """
  Subscribe to parameter change topics for tracked parameters.
  """
  @spec subscribe(module(), subscriptions()) :: :ok
  def subscribe(robot_module, subscriptions) do
    for param_path <- Map.keys(subscriptions) do
      PubSub.subscribe(robot_module, [:param | param_path])
    end

    :ok
  end

  @doc """
  Handle a parameter change message.

  Checks if the changed parameter is one we're subscribed to,
  re-resolves all options, and calls the provided callback with the new options.

  Returns `{:changed, new_resolved}` if the parameter was tracked,
  or `:ignored` if not.
  """
  @spec handle_change([atom()], subscriptions(), keyword(), module() | RobotState.t()) ::
          {:changed, keyword()} | :ignored
  def handle_change(param_path, subscriptions, raw_opts, robot_module_or_state) do
    if Map.has_key?(subscriptions, param_path) do
      {_subs, new_resolved} = resolve(raw_opts, robot_module_or_state)
      {:changed, new_resolved}
    else
      :ignored
    end
  end

  defp normalise_robot_arg(%RobotState{} = state) do
    {state.robot.name, state}
  end

  defp normalise_robot_arg(module) when is_atom(module) do
    {module, Runtime.get_robot_state(module)}
  end
end
