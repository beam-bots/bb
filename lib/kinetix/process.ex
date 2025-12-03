# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Process do
  @moduledoc """
  Helper functions for building child specs and looking up processes in the robot's registry.
  """

  @doc """
  Build a child_spec that registers the process in the robot's registry.

  The resulting child spec uses `GenServer.start_link/3` directly to ensure
  the process is registered with the correct `:via` tuple.

  The process is registered by its name (which must be globally unique across
  the robot). The full path is passed to the process in its init args for
  context, but is not used for registration.
  """
  @spec child_spec(module, atom, module | {module, Keyword.t()}, [atom]) :: map
  def child_spec(robot_module, name, user_child_spec, path) do
    {module, user_args} = normalize_child_spec(user_child_spec)
    full_path = path ++ [name]

    init_arg =
      Keyword.merge(user_args,
        kinetix: %{robot: robot_module, path: full_path}
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
