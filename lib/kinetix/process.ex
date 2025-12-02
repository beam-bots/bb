defmodule Kinetix.Process do
  @moduledoc """
  Helper functions for building child specs and looking up processes in the robot's registry.
  """

  @doc """
  Build a child_spec that registers the process in the robot's registry.

  The resulting child spec uses `GenServer.start_link/3` directly to ensure
  the process is registered with the correct `:via` tuple.
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
           [name: via(robot_module, full_path)]
         ]}
    }
  end

  @doc """
  Build a `:via` tuple for registry lookup.
  """
  @spec via(module, [atom]) :: {:via, module, {atom, [atom]}}
  def via(robot_module, path) do
    {:via, Registry, {registry_name(robot_module), path}}
  end

  @doc """
  Look up a process by path in the robot's registry.

  Returns `pid` if found, `:undefined` otherwise.
  """
  @spec whereis(module, [atom]) :: pid | :undefined
  def whereis(robot_module, path) do
    Registry.whereis_name({registry_name(robot_module), path})
  end

  @doc """
  Returns the registry name for a robot module.
  """
  @spec registry_name(module) :: atom
  def registry_name(robot_module) do
    Module.concat(robot_module, Registry)
  end

  defp normalize_child_spec(module) when is_atom(module), do: {module, []}
  defp normalize_child_spec({module, args}) when is_atom(module), do: {module, args}
end
