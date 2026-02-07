# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.PubSub do
  @moduledoc """
  Hierarchical pubsub system for robot component messages.

  Allows processes to subscribe to messages by path with optional message type
  filtering. Paths are prefixed with a source type atom (`:sensor`, `:actuator`,
  etc.) followed by the location path through the robot topology.

  ## Path Format

      [:sensor, :base_link, :joint1, :imu1]      # specific sensor
      [:actuator, :base_link, :joint1, :motor1]  # specific actuator

  ## Subscription Patterns

      # Exact match - only messages from this specific sensor
      subscribe(MyRobot, [:sensor, :base_link, :joint1, :imu1])

      # Subtree - all sensors under joint1
      subscribe(MyRobot, [:sensor, :base_link, :joint1])

      # All of type - all sensors anywhere
      subscribe(MyRobot, [:sensor])

      # All messages
      subscribe(MyRobot, [])

  ## Message Format

  Subscribers receive messages as:

      {:bb, source_path, %BB.Message{}}

  Where `source_path` is the full path of the publisher.

  ## Message Type Filtering

  Subscribe with `message_types` option to filter by payload type:

      subscribe(MyRobot, [:sensor], message_types: [BB.Message.Sensor.Imu])

  Empty list (default) means no filtering - receive all message types.
  """

  alias BB.Dsl.Info

  @doc """
  Returns the pubsub registry name for a robot module.
  """
  @spec registry_name(module) :: atom
  def registry_name(robot_module) do
    Module.concat(robot_module, PubSub)
  end

  @doc """
  Subscribe the calling process to messages matching the given path.

  ## Options

    * `:message_types` - List of message payload modules to receive. Empty list
      (default) means receive all message types.

  ## Examples

      # All IMU messages from sensors under joint1
      subscribe(MyRobot, [:sensor, :base_link, :joint1],
        message_types: [BB.Message.Sensor.Imu])

      # All sensor messages (no type filter)
      subscribe(MyRobot, [:sensor])

      # All messages from anywhere
      subscribe(MyRobot, [])
  """
  @spec subscribe(module, [atom], keyword) :: {:ok, pid} | {:error, term}
  def subscribe(robot, path, opts \\ []) when is_atom(robot) and is_list(path) do
    message_types = Keyword.get(opts, :message_types, [])
    settings = Info.settings(robot)
    settings.registry_module.register(registry_name(robot), path, message_types)
  end

  @doc """
  Unsubscribe the calling process from the given path.
  """
  @spec unsubscribe(module, [atom]) :: :ok
  def unsubscribe(robot, path) when is_atom(robot) and is_list(path) do
    settings = Info.settings(robot)
    settings.registry_module.unregister(registry_name(robot), path)
  end

  @doc """
  Publish a message to all matching subscribers.

  The message is dispatched to subscribers registered at the exact path and all
  ancestor paths. At each level, subscribers are filtered by their registered
  `message_types` (if any).

  ## Examples

      # From a sensor process
      path = [:sensor | state.bb.path]
      publish(state.bb.robot, path, message)
  """
  @spec publish(module, [atom], BB.Message.t()) :: :ok
  def publish(robot, path, %BB.Message{} = message)
      when is_atom(robot) and is_list(path) do
    settings = Info.settings(robot)
    registry_module = settings.registry_module
    registry = registry_name(robot)
    message_module = message.payload.__struct__

    path
    |> ancestor_paths()
    |> Enum.each(fn topic ->
      registry_module.dispatch(
        registry,
        topic,
        &dispatch_to_subscribers(&1, path, message, message_module)
      )
    end)

    :ok
  end

  defp dispatch_to_subscribers(entries, path, message, message_module) do
    for {pid, msg_types} <- entries,
        msg_types == [] or message_module in msg_types do
      send(pid, {:bb, path, message})
    end
  end

  @doc """
  List subscribers registered at a specific path.

  Returns a list of `{pid, message_types}` tuples. Useful for debugging.
  """
  @spec subscribers(module, [atom]) :: [{pid, [module]}]
  def subscribers(robot, path) when is_atom(robot) and is_list(path) do
    settings = Info.settings(robot)
    settings.registry_module.lookup(registry_name(robot), path)
  end

  @doc false
  def ancestor_paths(path) do
    path
    |> Enum.scan([], fn elem, acc -> acc ++ [elem] end)
    |> Enum.reverse()
    |> Kernel.++([[]])
  end
end
