# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Bb.FromUrdf do
    @shortdoc "Generate a BB robot module from a URDF file"

    @moduledoc """
    #{@shortdoc}

    Reads a URDF XML file and writes a `defmodule` that uses `BB` with an
    equivalent topology to your project.

    If the target module already exists, only its `topology do ... end` block
    is replaced — `settings`, sensors, controllers, commands and any other
    hand-written content are left in place. If the existing module has no
    `topology` block, one is appended.

    ## Example

    ```bash
    mix bb.from_urdf path/to/robot.urdf --module MyApp.Robot
    ```

    ## Options

      * `--module`, `-m` - The module name for the generated robot.
        Defaults to `{AppPrefix}.Robot`.

    ## URDF feature support

    `<mimic>` joints are emitted as a `BB.Sensor.Mimic` attached to the
    mimicking joint — BB's sensor implements the same
    `position * multiplier + offset` semantics as URDF.

    These URDF features have no direct BB equivalent and are skipped with a
    warning rather than failing the import:

      * `<safety_controller>` blocks
      * `<transmission>` blocks
      * `<gazebo>` extensions

    Mesh files are referenced as-is — `package://` URIs from ROS are not
    rewritten, so you may need to copy meshes into the project and adjust
    paths by hand.
    """

    use Igniter.Mix.Task

    alias BB.Urdf.{Importer, Parser}
    alias Igniter.Code.{Common, Function}

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        positional: [:urdf_path],
        schema: [module: :string],
        aliases: [m: :module]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      %{urdf_path: urdf_path} = igniter.args.positional
      module_name = resolve_module(igniter)

      case Parser.parse_file(urdf_path) do
        {:ok, parsed} ->
          generate(igniter, parsed, module_name)

        {:error, reason} ->
          Igniter.add_issue(igniter, "Could not parse URDF #{urdf_path}: #{inspect(reason)}")
      end
    end

    defp resolve_module(igniter) do
      case Keyword.get(igniter.args.options, :module) do
        nil -> Igniter.Project.Module.module_name(igniter, "Robot")
        name -> Igniter.Project.Module.parse(name)
      end
    end

    defp generate(igniter, parsed, module_name) do
      with {:ok, source, warnings} <- Importer.to_source(parsed, module_name),
           {:ok, topology_ast, _} <- Importer.to_topology_quoted(parsed) do
        igniter
        |> find_and_update_or_create_module(module_name, source, topology_ast)
        |> add_warnings(warnings)
      else
        {:error, reason} ->
          Igniter.add_issue(igniter, "Could not generate BB module: #{inspect(reason)}")
      end
    end

    defp find_and_update_or_create_module(igniter, module_name, source, topology_ast) do
      body = strip_defmodule(source, module_name)

      Igniter.Project.Module.find_and_update_or_create_module(
        igniter,
        module_name,
        body,
        fn zipper -> replace_or_insert_topology(zipper, topology_ast) end
      )
    end

    defp replace_or_insert_topology(zipper, topology_ast) do
      case Function.move_to_function_call_in_current_scope(zipper, :topology, 1) do
        {:ok, found} ->
          {:ok, Common.replace_code(found, topology_ast)}

        :error ->
          {:ok, Common.add_code(zipper, topology_ast)}
      end
    end

    defp strip_defmodule(source, module_name) do
      module_string = Macro.inspect_atom(:literal, module_name)

      source
      |> String.replace(~r/\Adefmodule\s+#{Regex.escape(module_string)}\s+do\n?/, "")
      |> String.replace(~r/\nend\s*\z/, "")
      |> String.trim_trailing()
    end

    defp add_warnings(igniter, warnings) do
      Enum.reduce(warnings, igniter, fn warning, acc ->
        Igniter.add_warning(acc, warning)
      end)
    end
  end
else
  defmodule Mix.Tasks.Bb.FromUrdf do
    @shortdoc "Generate a BB robot module from a URDF file"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb.from_urdf task requires igniter.

          mix deps.get
          mix bb.from_urdf path/to/robot.urdf --module MyApp.Robot
      """)

      exit({:shutdown, 1})
    end
  end
end
