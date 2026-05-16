# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule BB.Igniter do
    @moduledoc """
    Helpers for writing Igniter installers for BB add-on packages.

    Only available when `:igniter` is loaded.
    """

    alias Igniter.Code.Common
    alias Igniter.Code.Function

    @doc """
    Returns the robot module to operate on.

    Resolution order:

      1. The `--robot` option from `igniter.args.options` (parsed module name)
      2. `{AppPrefix}.Robot` (e.g. `MyApp.Robot`)

    Add `robot: :string` (and ideally `aliases: [r: :robot]`) to your task's
    schema to support the `--robot` flag.
    """
    @spec robot_module(Igniter.t()) :: module()
    def robot_module(igniter) do
      case Keyword.get(igniter.args.options, :robot) do
        nil -> Igniter.Project.Module.module_name(igniter, "Robot")
        name -> Igniter.Project.Module.parse(name)
      end
    end

    @doc """
    Adds a `controller` entry to the robot's `controllers do … end` section.

    `code` is the full DSL call as a string, e.g.

        controller :dynamixel, {BB.Servo.Robotis.Controller, port: ...}

    The section is created if it doesn't already exist. Idempotent on `name`:
    if a controller with the same name is already present, the igniter is
    returned unchanged.
    """
    @spec add_controller(Igniter.t(), module(), atom(), String.t()) :: Igniter.t()
    def add_controller(igniter, robot_module, name, code) do
      add_named_dsl_entity(igniter, robot_module, :controllers, :controller, name, code)
    end

    @doc """
    Adds a `bridge` entry to the robot's `parameters do … end` section.

    `code` is the full DSL call as a string, e.g.

        bridge :robotis_bridge, {BB.Servo.Robotis.Bridge, controller: :dynamixel}

    The section is created if it doesn't already exist. Idempotent on `name`.
    """
    @spec add_parameter_bridge(Igniter.t(), module(), atom(), String.t()) :: Igniter.t()
    def add_parameter_bridge(igniter, robot_module, name, code) do
      add_named_dsl_entity(igniter, robot_module, :parameters, :bridge, name, code)
    end

    @doc """
    Adds a nested `group` hierarchy to the robot's `parameters do … end` section.

    `group_path` is a list of atoms describing the path of nested groups to
    create, e.g. `[:config, :feetech]` produces:

        group :config do
          group :feetech do
            <body_code>
          end
        end

    `body_code` is the contents of the innermost group (typically one or more
    `param ...` declarations) as a string.

    The `parameters` section and intermediate groups are created as needed.
    Idempotent: if the full group path already exists, the body is not added a
    second time (so manually-edited param contents are preserved).
    """
    @spec add_param_group(Igniter.t(), module(), [atom(), ...], String.t()) :: Igniter.t()
    def add_param_group(igniter, robot_module, group_path, body_code)
        when is_list(group_path) and group_path != [] do
      code = wrap_in_groups(group_path, body_code)

      Spark.Igniter.update_dsl(igniter, robot_module, [{:section, :parameters}], nil, fn zipper ->
        if group_path_exists?(zipper, group_path) do
          {:ok, zipper}
        else
          {:ok, Common.add_code(zipper, code)}
        end
      end)
    end

    @doc """
    Ensures the robot's child spec in the application module carries the given
    opts.

    For new robot children, the opts are inserted directly. For existing
    children, the existing opts are replaced. This is a coarse operation; if
    multiple installers need to set different keys, the last one to run wins.
    """
    @spec set_robot_opts(Igniter.t(), module(), keyword()) :: Igniter.t()
    def set_robot_opts(igniter, robot_module, opts) do
      Igniter.Project.Application.add_new_child(igniter, {robot_module, opts},
        opts_updater: fn zipper -> {:ok, Sourceror.Zipper.replace(zipper, opts)} end
      )
    end

    @doc """
    Adds a `link` entry to the robot's `topology do … end` section.

    `body_code` is the DSL inside the link block as a string, e.g.

        visual do
          origin do z(~u(0.1 meter)) end
        end

        joint :shoulder do
          ...
        end

    The section is created if it doesn't already exist. Idempotent on `name`:
    if a top-level link with the same name is already present, the igniter is
    returned unchanged.
    """
    @spec add_topology_link(Igniter.t(), module(), atom(), String.t()) :: Igniter.t()
    def add_topology_link(igniter, robot_module, name, body_code) do
      code = "link :#{name} do\n#{indent(body_code)}\nend\n"
      add_named_dsl_entity(igniter, robot_module, :topology, :link, name, code)
    end

    @doc """
    Populates an existing empty `link` in the robot's topology with `body_code`.

    `link_path` is the chain of link names from the topology root down to the
    leaf to populate, e.g. `[:base_link]` or
    `[:base_link, :shoulder_link, :upper_arm_link]`.

    Idempotent: if the leaf link already has any DSL entities in its body,
    the igniter is returned unchanged so user customisations are preserved.
    If the leaf link is empty, `body_code` is inserted as its body.

    Returns the igniter unchanged if any link in `link_path` doesn't exist.
    """
    @spec populate_link(Igniter.t(), module(), [atom(), ...], String.t()) :: Igniter.t()
    def populate_link(igniter, robot_module, link_path, body_code)
        when is_list(link_path) and link_path != [] do
      Spark.Igniter.update_dsl(igniter, robot_module, [{:section, :topology}], nil, fn zipper ->
        maybe_insert_link_body(zipper, link_path, body_code)
      end)
    end

    defp maybe_insert_link_body(zipper, link_path, body_code) do
      case descend_to_link_body(zipper, link_path) do
        {:ok, body_zipper} -> insert_if_empty(zipper, body_zipper, body_code)
        :error -> {:ok, zipper}
      end
    end

    defp insert_if_empty(zipper, body_zipper, body_code) do
      if link_body_empty?(body_zipper) do
        {:ok, Common.add_code(body_zipper, body_code)}
      else
        {:ok, zipper}
      end
    end

    defp wrap_in_groups([leaf], body_code) do
      "group :#{leaf} do\n#{indent(body_code)}\nend\n"
    end

    defp wrap_in_groups([name | rest], body_code) do
      "group :#{name} do\n#{indent(wrap_in_groups(rest, body_code))}\nend\n"
    end

    defp indent(text) do
      text
      |> String.split("\n")
      |> Enum.map_join("\n", fn
        "" -> ""
        line -> "  " <> line
      end)
    end

    defp add_named_dsl_entity(igniter, robot_module, section, entity, name, code) do
      Spark.Igniter.update_dsl(igniter, robot_module, [{:section, section}], nil, fn zipper ->
        if entity_with_name_exists?(zipper, entity, name) do
          {:ok, zipper}
        else
          {:ok, Common.add_code(zipper, code)}
        end
      end)
    end

    defp entity_with_name_exists?(zipper, entity, name) do
      case Function.move_to_function_call_in_current_scope(
             zipper,
             entity,
             [2, 3],
             &Function.argument_equals?(&1, 0, name)
           ) do
        {:ok, _} -> true
        _ -> false
      end
    end

    defp group_path_exists?(_zipper, []), do: true

    defp group_path_exists?(zipper, [name | rest]) do
      with {:ok, group_zipper} <-
             Function.move_to_function_call_in_current_scope(
               zipper,
               :group,
               [2, 3],
               &Function.argument_equals?(&1, 0, name)
             ),
           {:ok, group_body} <- Common.move_to_do_block(group_zipper) do
        group_path_exists?(group_body, rest)
      else
        _ -> false
      end
    end

    defp descend_to_link_body(zipper, [name]) do
      case find_named_link(zipper, name) do
        {:ok, link_zipper} -> Common.move_to_do_block(link_zipper)
        :error -> :error
      end
    end

    defp descend_to_link_body(zipper, [name | rest]) do
      with {:ok, link_zipper} <- find_named_link(zipper, name),
           {:ok, body_zipper} <- Common.move_to_do_block(link_zipper) do
        descend_to_link_body(body_zipper, rest)
      else
        _ -> :error
      end
    end

    defp find_named_link(zipper, name) do
      case Function.move_to_function_call_in_current_scope(
             zipper,
             :link,
             [2, 3],
             &Function.argument_equals?(&1, 0, name)
           ) do
        {:ok, link_zipper} -> {:ok, link_zipper}
        _ -> :error
      end
    end

    defp link_body_empty?(body_zipper) do
      case Sourceror.Zipper.node(body_zipper) do
        nil -> true
        {:__block__, _, []} -> true
        {:__block__, _, [nil]} -> true
        _ -> false
      end
    end
  end
end
