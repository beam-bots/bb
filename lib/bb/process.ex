# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Process do
  @moduledoc """
  Helper functions for building child specs and looking up processes in the robot's registry.
  """

  @type process_type :: :actuator | :sensor | :controller

  @doc """
  Build a child_spec that registers the process in the robot's registry.

  The resulting child spec uses the appropriate wrapper GenServer based on type:
  - `:actuator` → `BB.Actuator.Server` (or `BB.Sim.Actuator` in simulation mode)
  - `:sensor` → `BB.Sensor.Server`
  - `:controller` → `BB.Controller.Server`

  The user's callback module is passed via `__callback_module__` and the wrapper
  server delegates GenServer callbacks to it.

  The process is registered by its name (which must be globally unique across
  the robot). The full path is passed to the process in its init args for
  context, but is not used for registration.

  ## Options

  - `:simulation` - when set (e.g., `:kinematic`), actuators use `BB.Sim.Actuator`
    instead of the real actuator module
  """
  @spec child_spec(
          module,
          atom,
          module | {module, Keyword.t()},
          [atom],
          process_type,
          Keyword.t()
        ) ::
          map
  def child_spec(robot_module, name, user_child_spec, path, type, opts \\ []) do
    simulation_mode = Keyword.get(opts, :simulation)

    if simulation_mode && type == :actuator do
      build_simulated_actuator_spec(robot_module, name, path)
    else
      build_real_child_spec(robot_module, name, user_child_spec, path, type)
    end
  end

  defp build_real_child_spec(robot_module, name, user_child_spec, path, type) do
    {callback_module, user_args} = normalize_child_spec(user_child_spec)
    full_path = path ++ [name]

    wrapper_module = wrapper_for_type(type)

    init_arg =
      user_args
      |> Keyword.put(:bb, %{robot: robot_module, path: full_path})
      |> Keyword.put(:__callback_module__, callback_module)

    %{
      id: name,
      start:
        {wrapper_module, :start_link,
         [
           init_arg,
           [name: via(robot_module, name)]
         ]}
    }
  end

  defp build_simulated_actuator_spec(robot_module, name, path) do
    full_path = path ++ [name]

    init_arg = [
      bb: %{robot: robot_module, path: full_path},
      __callback_module__: BB.Sim.Actuator
    ]

    %{
      id: name,
      start:
        {BB.Actuator.Server, :start_link,
         [
           init_arg,
           [name: via(robot_module, name)]
         ]}
    }
  end

  defp wrapper_for_type(:actuator), do: BB.Actuator.Server
  defp wrapper_for_type(:sensor), do: BB.Sensor.Server
  defp wrapper_for_type(:controller), do: BB.Controller.Server
  defp wrapper_for_type(:bridge), do: nil

  @doc """
  Build a child_spec for a bridge process.

  Bridges use GenServer directly (not a wrapper) as they implement the full
  GenServer behaviour themselves.
  """
  @spec bridge_child_spec(module, atom, module | {module, Keyword.t()}, [atom]) :: map
  def bridge_child_spec(robot_module, name, user_child_spec, path) do
    {module, user_args} = normalize_child_spec(user_child_spec)
    full_path = path ++ [name]

    init_arg =
      Keyword.merge(user_args,
        bb: %{robot: robot_module, path: full_path}
      )

    %{
      id: name,
      start:
        {GenServer, :start_link,
         [
           module,
           init_arg,
           [name: via(robot_module, name)]
         ]}
    }
  end

  @doc """
  Build a `:via` tuple for registry lookup by name.
  """
  @spec via(module, atom) :: {:via, module, {atom, atom}}
  def via(robot_module, name) do
    {:via, Registry, {registry_name(robot_module), name}}
  end

  @doc """
  Look up a process by name in the robot's registry.

  Returns `pid` if found, `:undefined` otherwise.
  """
  @spec whereis(module, atom) :: pid | :undefined
  def whereis(robot_module, name) do
    Registry.whereis_name({registry_name(robot_module), name})
  end

  @doc """
  Returns the registry name for a robot module.
  """
  @spec registry_name(module) :: atom
  def registry_name(robot_module) do
    Module.concat(robot_module, Registry)
  end

  @doc """
  Cast a message to a process looked up by name.

  Uses a `:via` tuple so the registry handles lookup atomically.
  Returns `:ok` (GenServer.cast always returns :ok, even if process doesn't exist).
  """
  @spec cast(module, atom, term) :: :ok
  def cast(robot_module, name, message) do
    GenServer.cast(via(robot_module, name), message)
  end

  @doc """
  Call a process looked up by name.

  Uses a `:via` tuple so the registry handles lookup atomically.
  Raises if the process doesn't exist or times out.
  """
  @spec call(module, atom, term, timeout) :: term
  def call(robot_module, name, message, timeout \\ 5000) do
    GenServer.call(via(robot_module, name), message, timeout)
  end

  @doc """
  Send a raw message to a process looked up by name.

  Uses `Registry.dispatch/3` to handle lookup atomically.
  Returns `:ok` regardless of whether the process exists.
  """
  @spec send(module, atom, term) :: :ok
  def send(robot_module, name, message) do
    Registry.dispatch(registry_name(robot_module), name, fn entries ->
      for {pid, _value} <- entries, do: Kernel.send(pid, message)
    end)
  end

  defp normalize_child_spec(module) when is_atom(module), do: {module, []}
  defp normalize_child_spec({module, args}) when is_atom(module), do: {module, args}
end
