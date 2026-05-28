# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ValidateChildSpecBehavioursTransformer do
  @moduledoc """
  Enforces that every module wired into a component slot in the topology DSL
  implements the matching BB component behaviour.

  Spark's built-in `{:behaviour, X}` schema type only validates that the
  value is an atom — it does not check behaviour conformance — so a bare
  `use GenServer` module could otherwise be wired in as e.g. an actuator,
  passing DSL compilation only to crash at runtime when the server expects
  callbacks (`options_schema/0`, `init/1`, etc.) that aren't defined.

  This check lives in a transformer (rather than a verifier) so it raises
  at compile time. Verifiers run in `@after_verify` where Elixir does not
  tolerate errors, so they can only warn.
  """

  use Spark.Dsl.Transformer

  alias BB.Dsl.{Actuator, Bridge, Controller, Estimator, Joint, Link, Sensor}
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(BB.Dsl.UniquenessTransformer), do: true
  def after?(_), do: false

  @impl true
  def before?(BB.Dsl.RobotTransformer), do: true
  def before?(_), do: false

  @impl true
  def transform(dsl) do
    robot_module = Transformer.get_persisted(dsl, :module)

    with :ok <- check_controllers(dsl, robot_module),
         :ok <- check_robot_sensors(dsl, robot_module),
         :ok <- check_bridges(dsl, robot_module),
         :ok <- check_topology(dsl, robot_module) do
      {:ok, dsl}
    end
  end

  defp check_controllers(dsl, robot_module) do
    dsl
    |> Transformer.get_entities([:controllers])
    |> reduce_check(robot_module, BB.Controller, fn %Controller{name: name} ->
      [:controllers, name]
    end)
  end

  defp check_robot_sensors(dsl, robot_module) do
    dsl
    |> Transformer.get_entities([:sensors])
    |> reduce_check(robot_module, BB.Sensor, fn %Sensor{name: name} ->
      [:sensors, name]
    end)
  end

  defp check_bridges(dsl, robot_module) do
    dsl
    |> Transformer.get_entities([:parameters])
    |> Enum.filter(&is_struct(&1, Bridge))
    |> reduce_check(robot_module, BB.Bridge, fn %Bridge{name: name} ->
      [:parameters, name]
    end)
  end

  defp check_topology(dsl, robot_module) do
    dsl
    |> Transformer.get_entities([:topology])
    |> reduce_topology([], robot_module)
  end

  defp reduce_topology(entities, path, robot_module) do
    Enum.reduce_while(entities, :ok, fn entity, :ok ->
      entity
      |> check_topology_entity(path, robot_module)
      |> to_reduce_acc()
    end)
  end

  defp check_topology_entity(%Link{} = link, path, robot_module) do
    link_path = path ++ [:topology, :link, link.name]

    with :ok <-
           reduce_check(link.sensors, robot_module, BB.Sensor, fn %Sensor{name: name} ->
             link_path ++ [:sensor, name]
           end),
         :ok <- check_estimators_with_nested(link.sensors, link_path, robot_module),
         :ok <-
           reduce_check(link.estimators, robot_module, BB.Estimator, fn %Estimator{name: name} ->
             link_path ++ [:estimator, name]
           end) do
      reduce_topology(link.joints, link_path, robot_module)
    end
  end

  defp check_topology_entity(%Joint{} = joint, path, robot_module) do
    joint_path = path ++ [:joint, joint.name]
    sensors = Map.get(joint, :sensors, [])
    actuators = Map.get(joint, :actuators, [])

    with :ok <-
           reduce_check(sensors, robot_module, BB.Sensor, fn %Sensor{name: name} ->
             joint_path ++ [:sensor, name]
           end),
         :ok <- check_estimators_with_nested(sensors, joint_path, robot_module),
         :ok <-
           reduce_check(actuators, robot_module, BB.Actuator, fn %Actuator{name: name} ->
             joint_path ++ [:actuator, name]
           end) do
      case Map.get(joint, :link) do
        nil -> :ok
        nested -> check_topology_entity(nested, path, robot_module)
      end
    end
  end

  defp check_topology_entity(_entity, _path, _robot_module), do: :ok

  defp check_estimators_with_nested(sensors, parent_path, robot_module) do
    Enum.reduce_while(sensors, :ok, fn %Sensor{} = sensor, :ok ->
      sensor_path = parent_path ++ [:sensor, sensor.name]
      path_fun = fn %Estimator{name: name} -> sensor_path ++ [:estimator, name] end

      sensor.estimators
      |> reduce_check(robot_module, BB.Estimator, path_fun)
      |> to_reduce_acc()
    end)
  end

  defp reduce_check(entities, robot_module, behaviour, path_fun) do
    Enum.reduce_while(entities, :ok, fn entity, :ok ->
      entity.child_spec
      |> check_one(path_fun.(entity), robot_module, behaviour)
      |> to_reduce_acc()
    end)
  end

  defp to_reduce_acc(:ok), do: {:cont, :ok}
  defp to_reduce_acc({:error, _} = error), do: {:halt, error}

  defp check_one(child_spec, path, robot_module, behaviour) do
    {module, _opts} = normalize(child_spec)

    # `ensure_compiled` asks Mix to finish compiling the module first when it's
    # part of the current project, so we don't race with in-tree fixtures that
    # reference a sensor/actuator defined elsewhere in the same package. If
    # the module genuinely can't be located, defer to runtime — the component
    # server will refuse to start a module that doesn't implement the
    # expected behaviour. Halting compilation here would block legitimate
    # cross-package CI matrices where the referenced module isn't yet
    # loadable from the transformer's vantage point.
    case Code.ensure_compiled(module) do
      {:module, _} ->
        if behaviour in declared_behaviours(module) do
          :ok
        else
          {:error,
           DslError.exception(
             module: robot_module,
             path: path,
             message: """
             Module #{inspect(module)} must implement the #{inspect(behaviour)} behaviour.

             Add `use #{inspect(behaviour)}` (or `@behaviour #{inspect(behaviour)}` and the required callbacks) to #{inspect(module)}.
             """
           )}
        end

      {:error, _reason} ->
        :ok
    end
  end

  defp normalize(module) when is_atom(module), do: {module, []}
  defp normalize({module, opts}) when is_atom(module), do: {module, opts}

  defp declared_behaviours(module) do
    module.module_info(:attributes)
    |> Keyword.get_values(:behaviour)
    |> List.flatten()
  end
end
