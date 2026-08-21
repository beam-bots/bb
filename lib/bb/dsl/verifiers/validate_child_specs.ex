# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidateChildSpecs do
  @moduledoc """
  Validates that child_spec options match the module's `options_schema/0`.

  Behaviour conformance is enforced separately (and at hard-error severity)
  by `BB.Dsl.ValidateChildSpecBehavioursTransformer`; this verifier assumes
  every wired-in module is already a proper component and so always has
  `options_schema/0` defined.

  Options containing `param()` references are skipped from validation as they
  will be resolved at runtime.
  """

  use Spark.Dsl.Verifier

  alias BB.Dsl.{Actuator, Bridge, ChildSpecOptions, Controller, Estimator, Joint, Link, Sensor}
  alias Spark.Dsl.Verifier
  alias Spark.Error.DslError

  @impl true
  def verify(dsl_state) do
    module = Verifier.get_persisted(dsl_state, :module)

    with :ok <- verify_controllers(dsl_state, module),
         :ok <- verify_robot_sensors(dsl_state, module),
         :ok <- verify_topology(dsl_state, module) do
      verify_bridges(dsl_state, module)
    end
  end

  defp verify_controllers(dsl_state, robot_module) do
    dsl_state
    |> Verifier.get_entities([:controllers])
    |> Enum.reduce_while(:ok, fn %Controller{} = controller, :ok ->
      case validate_child_spec(
             controller.child_spec,
             [:controllers, controller.name],
             robot_module
           ) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_robot_sensors(dsl_state, robot_module) do
    dsl_state
    |> Verifier.get_entities([:sensors])
    |> Enum.reduce_while(:ok, fn %Sensor{} = sensor, :ok ->
      case validate_child_spec(
             sensor.child_spec,
             [:sensors, sensor.name],
             robot_module
           ) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_bridges(dsl_state, robot_module) do
    dsl_state
    |> Verifier.get_entities([:parameters])
    |> Enum.filter(&is_struct(&1, Bridge))
    |> Enum.reduce_while(:ok, fn %Bridge{} = bridge, :ok ->
      case validate_child_spec(
             bridge.child_spec,
             [:parameters, bridge.name],
             robot_module
           ) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_topology(dsl_state, robot_module) do
    dsl_state
    |> Verifier.get_entities([:topology])
    |> verify_topology_entities([], robot_module)
  end

  defp verify_topology_entities(entities, path, robot_module) do
    Enum.reduce_while(entities, :ok, fn entity, :ok ->
      case verify_topology_entity(entity, path, robot_module) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_topology_entity(%Link{} = link, path, robot_module) do
    link_path = path ++ [:topology, :link, link.name]

    with :ok <- verify_link_sensors(link, link_path, robot_module),
         :ok <- verify_link_estimators(link, link_path, robot_module) do
      verify_topology_entities(link.joints, link_path, robot_module)
    end
  end

  defp verify_topology_entity(%Joint{} = joint, path, robot_module) do
    joint_path = path ++ [:joint, joint.name]

    with :ok <- verify_joint_sensors(joint, joint_path, robot_module),
         :ok <- verify_joint_actuators(joint, joint_path, robot_module) do
      verify_nested_link(joint, path, robot_module)
    end
  end

  defp verify_topology_entity(_entity, _path, _robot_module), do: :ok

  defp verify_link_sensors(%Link{sensors: sensors}, path, robot_module) do
    Enum.reduce_while(sensors, :ok, fn %Sensor{} = sensor, :ok ->
      sensor_path = path ++ [:sensor, sensor.name]

      with :ok <- validate_child_spec(sensor.child_spec, sensor_path, robot_module),
           :ok <- verify_estimators(sensor.estimators, sensor_path, robot_module) do
        {:cont, :ok}
      else
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_link_estimators(%Link{estimators: estimators}, path, robot_module) do
    verify_estimators(estimators, path, robot_module)
  end

  defp verify_estimators(estimators, path, robot_module) do
    Enum.reduce_while(estimators, :ok, fn %Estimator{} = est, :ok ->
      est_path = path ++ [:estimator, est.name]

      case validate_child_spec(est.child_spec, est_path, robot_module) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_joint_sensors(%Joint{} = joint, path, robot_module) do
    sensors = Map.get(joint, :sensors, [])

    Enum.reduce_while(sensors, :ok, fn %Sensor{} = sensor, :ok ->
      sensor_path = path ++ [:sensor, sensor.name]

      with :ok <- validate_child_spec(sensor.child_spec, sensor_path, robot_module),
           :ok <- verify_estimators(sensor.estimators, sensor_path, robot_module) do
        {:cont, :ok}
      else
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_joint_actuators(%Joint{} = joint, path, robot_module) do
    actuators = Map.get(joint, :actuators, [])

    Enum.reduce_while(actuators, :ok, fn %Actuator{} = actuator, :ok ->
      actuator_path = path ++ [:actuator, actuator.name]

      case validate_child_spec(actuator.child_spec, actuator_path, robot_module) do
        :ok -> {:cont, :ok}
        {:error, error} -> {:halt, {:error, error}}
      end
    end)
  end

  defp verify_nested_link(%Joint{} = joint, path, robot_module) do
    case Map.get(joint, :link) do
      nil -> :ok
      nested_link -> verify_topology_entity(nested_link, path, robot_module)
    end
  end

  defp validate_child_spec(child_spec, path, robot_module) do
    {module, opts} = ChildSpecOptions.module_and_options(child_spec)
    validate_options(module, opts, path, robot_module)
  end

  defp validate_options(module, opts, path, robot_module) do
    schema = module.options_schema()

    case ChildSpecOptions.validate(module, opts) do
      {:ok, _validated} ->
        :ok

      {:error, %Spark.Options.ValidationError{} = error} ->
        {:error,
         DslError.exception(
           module: robot_module,
           path: path,
           message: """
           Invalid options for #{inspect(module)}:

           #{Exception.message(error)}

           Expected schema:
           #{format_schema(schema)}
           """
         )}
    end
  end

  defp format_schema(%Spark.Options{schema: schema}) do
    format_schema(schema)
  end

  defp format_schema(schema) when is_list(schema) do
    Enum.map_join(schema, "\n", fn {key, opts} ->
      type = Keyword.get(opts, :type, :any)
      required = if Keyword.get(opts, :required, false), do: " (required)", else: ""

      default =
        if Keyword.has_key?(opts, :default),
          do: " [default: #{inspect(Keyword.get(opts, :default))}]",
          else: ""

      doc = if opts[:doc], do: " - #{opts[:doc]}", else: ""

      "  #{key}: #{inspect(type)}#{required}#{default}#{doc}"
    end)
  end
end
