# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Bb.Install do
    @shortdoc "Installs BB into a project"
    @moduledoc """
    #{@shortdoc}

    Composes `bb.add_robot` (scaffolds a robot module) and `bb.add_nx_backend`
    (prompts to configure an Nx backend — defaults to EXLA).

    ## Example

    ```bash
    mix igniter.install bb
    mix igniter.install bb --robot MyApp.Robot --backend exla
    ```

    ## Options

    * `--robot` - The module name for the robot (defaults to `{AppPrefix}.Robot`).
    * `--backend` - Nx backend to install: `exla`, `torchx`, or `binary`.
      If omitted, the user is prompted (defaults to `exla` in non-interactive
      runs).
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        composes: ["bb.add_robot", "bb.add_nx_backend"],
        schema: [robot: :string, backend: :string],
        aliases: [r: :robot, b: :backend]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      robot_module = BB.Igniter.robot_module(igniter)

      igniter
      |> Formatter.import_dep(:bb)
      |> Igniter.compose_task("bb.add_robot", ["--robot", inspect(robot_module)])
      |> Igniter.compose_task("bb.add_nx_backend", nx_backend_argv(igniter))
    end

    defp nx_backend_argv(igniter) do
      case Keyword.get(igniter.args.options, :backend) do
        nil -> []
        backend -> ["--backend", backend]
      end
    end
  end
else
  defmodule Mix.Tasks.Bb.Install do
    @shortdoc "Installs BB into a project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb.install task requires igniter. Please install igniter and try again.

          mix igniter.install bb
      """)

      exit({:shutdown, 1})
    end
  end
end
