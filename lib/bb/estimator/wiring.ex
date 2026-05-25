# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.Wiring do
  @moduledoc """
  Builds the supervisor `child_spec` for an `estimator` DSL entity.

  Used by `BB.LinkSupervisor` for link-nested estimators and by
  `BB.Sensor.Server` for sensor-nested estimators. Centralises:

  - Output-path resolution (auto-derived `:out` plus explicit `output`
    overrides).
  - Input-spec resolution (sensor-nested: synthesised single implicit
    input pointing at the parent sensor's path; link-nested: passed
    through from the entity's declared `input` blocks).
  - `sync_tolerance` unit conversion to nanoseconds.
  - Construction of the `BB.Estimator.Context` delivered to the user
    module's `init/1`.
  - Construction of the `BB.Process.via/2` registration tuple so each
    estimator is addressable in the robot's registry.
  """

  alias BB.Dsl.Estimator
  alias BB.Estimator.Context
  alias BB.Robot.Units

  @typedoc "Path components leading to the parent supervisor (link path)."
  @type parent_path :: [atom()]

  @typedoc "Path identifying the parent publisher (sensor path) for sensor-nested estimators."
  @type parent_sensor_path :: [atom()]

  @doc """
  Builds a child spec for a link-nested estimator. `link_path` is the
  sequence of link names leading to the parent link (e.g.
  `[:base_link, :arm]`).
  """
  @spec link_nested_child_spec(module(), Estimator.t(), parent_path(), Keyword.t()) :: map()
  def link_nested_child_spec(robot_module, %Estimator{} = est, link_path, opts) do
    estimator_path = link_path ++ [est.name]
    full_path = [:estimator | estimator_path]
    target_frame = List.last(link_path)

    inputs = link_nested_inputs(est)
    outputs = build_outputs(est, full_path)

    context = %Context{
      robot: robot_module,
      path: full_path,
      target_frame: target_frame,
      transforms: %{}
    }

    build_child_spec(robot_module, est, full_path, inputs, outputs, context, opts)
  end

  @doc """
  Builds a child spec for a sensor-nested estimator. `parent_sensor_path`
  is the parent sensor's full publish path including the leading
  `:sensor` atom (e.g. `[:sensor, :base_link, :imu]`).
  """
  @spec sensor_nested_child_spec(
          module(),
          Estimator.t(),
          parent_sensor_path(),
          atom(),
          Keyword.t()
        ) :: map()
  def sensor_nested_child_spec(
        robot_module,
        %Estimator{} = est,
        parent_sensor_path,
        target_frame,
        opts
      ) do
    full_path = parent_sensor_path ++ [est.name]

    inputs = %{
      mode: :single,
      inputs: [%{name: :parent, path: parent_sensor_path, driver?: true}],
      sync_tolerance_ns: nil
    }

    outputs = build_outputs(est, full_path)

    context = %Context{
      robot: robot_module,
      path: full_path,
      target_frame: target_frame,
      transforms: %{}
    }

    build_child_spec(robot_module, est, full_path, inputs, outputs, context, opts)
  end

  # ----------------------------------------------------------------------------
  # Internals
  # ----------------------------------------------------------------------------

  defp link_nested_inputs(%Estimator{inputs: declared} = est) do
    specs =
      Enum.map(declared, fn input ->
        %{name: input.name, path: input.path, driver?: input.driver?}
      end)

    %{
      mode: if(length(declared) <= 1, do: :single, else: :multi),
      inputs: specs,
      sync_tolerance_ns: sync_tolerance_to_ns(est.sync_tolerance)
    }
  end

  defp sync_tolerance_to_ns(nil), do: nil

  defp sync_tolerance_to_ns(unit) do
    unit
    |> Localize.Unit.convert!("second")
    |> Units.extract_float()
    |> Kernel.*(1_000_000_000)
    |> round()
  end

  defp build_outputs(%Estimator{outputs: declared}, full_path) do
    declared_map =
      Enum.into(declared, %{}, fn output ->
        path = output.path || full_path
        {output.name, path}
      end)

    Map.put_new(declared_map, :out, full_path)
  end

  defp build_child_spec(
         robot_module,
         %Estimator{} = est,
         full_path,
         inputs,
         outputs,
         context,
         opts
       ) do
    {callback_module, user_args} = normalise(est.child_spec)
    estimator_name = est.name

    init_arg =
      user_args
      |> Keyword.put(:bb, %{robot: robot_module, path: full_path})
      |> Keyword.put(:estimator_context, context)
      |> Keyword.put(:__callback_module__, callback_module)
      |> Keyword.put(:__estimator_inputs__, inputs)
      |> Keyword.put(:__estimator_outputs__, outputs)
      |> Keyword.merge(passthrough_opts(opts))

    %{
      id: estimator_name,
      start:
        {BB.Estimator.Server, :start_link,
         [init_arg, [name: BB.Process.via(robot_module, estimator_name)]]}
    }
  end

  defp normalise(module) when is_atom(module), do: {module, []}
  defp normalise({module, args}) when is_atom(module), do: {module, args}

  # Pass through known process-level opts (e.g. `:simulation`). Filters out
  # robot-level keys that would otherwise leak into the user module's
  # resolved opts.
  defp passthrough_opts(opts) do
    opts
    |> Keyword.take([:simulation])
  end
end
