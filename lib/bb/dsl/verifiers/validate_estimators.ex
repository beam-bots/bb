# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidateEstimators do
  @moduledoc """
  Validates `estimator` entities declared in the DSL.

  Checks:

  - Sensor-nested estimators carry no `input` declarations (the parent
    sensor is the implicit input).
  - Link-nested estimators declare at least one input.
  - Multi-input estimators declare exactly one driver input.
  - Single-input link-nested estimators with `driver?: true` set on the
    sole input are accepted (treated as multi-input degenerate form).
  - Every `input` path resolves to an existing publisher (sensor or
    estimator) in the topology.
  - No cycles in the estimator dependency graph.
  - `sync_tolerance` is only declared on multi-input estimators (a stray
    `sync_tolerance` on a single-input estimator is a likely mistake).

  Path resolution is intentionally loose at this stage: any path beginning
  with `[:sensor, ...]` or `[:estimator, ...]` that matches a declared
  publisher's full path is accepted. Subtree subscriptions and external
  paths are not supported (use a controller-level subscription instead).
  """

  use Spark.Dsl.Verifier

  alias BB.Dsl.Estimator
  alias BB.Dsl.Estimator.Input
  alias BB.Dsl.Joint
  alias BB.Dsl.Link
  alias BB.Dsl.Sensor
  alias Spark.Dsl.Verifier
  alias Spark.Error.DslError

  @impl true
  def verify(dsl_state) do
    robot = Verifier.get_persisted(dsl_state, :module)
    catalogue = build_catalogue(dsl_state)

    with :ok <- validate_estimator_shapes(catalogue, robot),
         :ok <- validate_input_paths(catalogue, robot) do
      validate_no_cycles(catalogue, robot)
    end
  end

  # ----------------------------------------------------------------------------
  # Catalogue: a flat list of every declared estimator + the set of valid
  # publisher paths (sensor paths and estimator paths) we can resolve
  # inputs against.
  # ----------------------------------------------------------------------------

  defp build_catalogue(dsl_state) do
    {estimators, publisher_paths} =
      dsl_state
      |> Verifier.get_entities([:sensors])
      |> collect_sensors([], MapSet.new(), [])

    {estimators, publisher_paths} =
      dsl_state
      |> Verifier.get_entities([:topology])
      |> collect_topology([], publisher_paths, estimators)

    %{estimators: estimators, publisher_paths: publisher_paths}
  end

  defp collect_topology(entities, link_path, paths, estimators) do
    Enum.reduce(entities, {estimators, paths}, fn entity, {ests, ps} ->
      collect_topology_entity(entity, link_path, ps, ests)
    end)
  end

  defp collect_topology_entity(%Link{} = link, link_path, paths, estimators) do
    new_link_path = link_path ++ [link.name]

    {estimators, paths} =
      collect_sensors(link.sensors, new_link_path, paths, estimators)

    {estimators, paths} =
      collect_link_estimators(link.estimators, new_link_path, paths, estimators)

    collect_topology(link.joints, new_link_path, paths, estimators)
  end

  defp collect_topology_entity(%Joint{} = joint, link_path, paths, estimators) do
    joint_path = link_path ++ [joint.name]

    {estimators, paths} =
      collect_sensors(Map.get(joint, :sensors, []), joint_path, paths, estimators)

    case Map.get(joint, :link) do
      nil -> {estimators, paths}
      nested -> collect_topology_entity(nested, joint_path, paths, estimators)
    end
  end

  defp collect_topology_entity(_other, _link_path, paths, estimators), do: {estimators, paths}

  defp collect_sensors(sensors, link_path, paths, estimators) do
    Enum.reduce(sensors, {estimators, paths}, fn %Sensor{} = sensor, {ests, ps} ->
      sensor_path = [:sensor] ++ link_path ++ [sensor.name]
      ps = MapSet.put(ps, sensor_path)

      Enum.reduce(sensor.estimators, {ests, ps}, fn %Estimator{} = est, {ests2, ps2} ->
        est_path = sensor_path ++ [est.name]

        entry = %{
          estimator: est,
          mode: :sensor_nested,
          path: est_path,
          parent_sensor_path: sensor_path,
          link_path: link_path
        }

        {[entry | ests2], MapSet.put(ps2, est_path)}
      end)
    end)
  end

  defp collect_link_estimators(estimators, link_path, paths, accumulator) do
    Enum.reduce(estimators, {accumulator, paths}, fn %Estimator{} = est, {acc, ps} ->
      est_path = [:estimator] ++ link_path ++ [est.name]

      entry = %{
        estimator: est,
        mode: :link_nested,
        path: est_path,
        parent_sensor_path: nil,
        link_path: link_path
      }

      {[entry | acc], MapSet.put(ps, est_path)}
    end)
  end

  # ----------------------------------------------------------------------------
  # Shape validation
  # ----------------------------------------------------------------------------

  defp validate_estimator_shapes(%{estimators: estimators}, robot) do
    Enum.reduce_while(estimators, :ok, fn entry, :ok ->
      case validate_shape(entry, robot) do
        :ok -> {:cont, :ok}
        {:error, _} = err -> {:halt, err}
      end
    end)
  end

  defp validate_shape(%{mode: :sensor_nested, estimator: est} = entry, robot)
       when est.inputs != [] do
    {:error,
     error(
       robot,
       entry,
       "must not declare `input` blocks (sensor-nested estimators take their parent sensor's output implicitly)"
     )}
  end

  defp validate_shape(%{mode: :sensor_nested}, _robot), do: :ok

  defp validate_shape(%{mode: :link_nested, estimator: est} = entry, robot)
       when est.inputs == [] do
    {:error,
     error(
       robot,
       entry,
       "must declare at least one `input` (link-nested estimators have no implicit input)"
     )}
  end

  defp validate_shape(%{mode: :link_nested, estimator: est} = entry, robot) do
    drivers = Enum.filter(est.inputs, & &1.driver?)
    input_count = length(est.inputs)

    cond do
      input_count > 1 and drivers == [] ->
        {:error,
         error(
           robot,
           entry,
           "has #{input_count} inputs but no driver. Mark exactly one `input` with `driver?: true`."
         )}

      length(drivers) > 1 ->
        names = Enum.map(drivers, & &1.name)

        {:error,
         error(
           robot,
           entry,
           "has multiple driver inputs: #{inspect(names)}. Exactly one input may be marked `driver?: true`."
         )}

      input_count == 1 and est.sync_tolerance != nil ->
        {:error,
         error(
           robot,
           entry,
           "has a single input but declares `sync_tolerance`. Sync tolerance only applies to multi-input fan-in."
         )}

      true ->
        :ok
    end
  end

  # ----------------------------------------------------------------------------
  # Input path resolution
  # ----------------------------------------------------------------------------

  defp validate_input_paths(%{estimators: estimators, publisher_paths: paths}, robot) do
    Enum.reduce_while(estimators, :ok, fn entry, :ok ->
      case validate_inputs(entry, paths, robot) do
        :ok -> {:cont, :ok}
        {:error, _} = err -> {:halt, err}
      end
    end)
  end

  defp validate_inputs(%{estimator: %{inputs: inputs}} = entry, paths, robot) do
    Enum.reduce_while(inputs, :ok, fn %Input{} = input, :ok ->
      cond do
        input.path == entry.path ->
          {:halt,
           {:error,
            error(
              robot,
              entry,
              "input #{inspect(input.name)} references the estimator's own path #{inspect(input.path)} (self-cycle)"
            )}}

        MapSet.member?(paths, input.path) ->
          {:cont, :ok}

        true ->
          {:halt,
           {:error,
            error(
              robot,
              entry,
              "input #{inspect(input.name)} references unknown path #{inspect(input.path)}. " <>
                "Declare it as a sensor or estimator first."
            )}}
      end
    end)
  end

  # ----------------------------------------------------------------------------
  # Cycle detection over the input-dependency graph.
  # ----------------------------------------------------------------------------

  defp validate_no_cycles(%{estimators: estimators}, robot) do
    by_path = Enum.into(estimators, %{}, fn entry -> {entry.path, entry} end)

    Enum.reduce_while(estimators, :ok, fn entry, :ok ->
      case detect_cycle(entry, by_path, [entry.path], MapSet.new([entry.path])) do
        :ok -> {:cont, :ok}
        {:error, _} = err -> {:halt, err}
      end
    end)
    |> case do
      :ok ->
        :ok

      {:error, {entry, cycle_path}} ->
        {:error, error(robot, entry, "input dependency cycle: " <> format_cycle(cycle_path))}
    end
  end

  defp detect_cycle(entry, by_path, trail, visited) do
    Enum.reduce_while(entry.estimator.inputs, :ok, fn %Input{} = input, :ok ->
      visit_input(input, by_path, trail, visited, entry)
    end)
  end

  defp visit_input(%Input{} = input, by_path, trail, visited, entry) do
    cond do
      not Map.has_key?(by_path, input.path) ->
        {:cont, :ok}

      MapSet.member?(visited, input.path) ->
        {:halt, {:error, {entry, trail ++ [input.path]}}}

      true ->
        next = Map.fetch!(by_path, input.path)

        case detect_cycle(next, by_path, trail ++ [input.path], MapSet.put(visited, input.path)) do
          :ok -> {:cont, :ok}
          {:error, _} = err -> {:halt, err}
        end
    end
  end

  defp format_cycle(paths) do
    Enum.map_join(paths, " -> ", &inspect/1)
  end

  # ----------------------------------------------------------------------------
  # Error helpers
  # ----------------------------------------------------------------------------

  defp error(robot, entry, message) do
    DslError.exception(
      module: robot,
      path: entry.path,
      message: "estimator #{inspect(entry.estimator.name)} at #{inspect(entry.path)}: " <> message
    )
  end
end
