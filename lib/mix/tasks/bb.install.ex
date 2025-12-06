# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Bb.Install do
    @shortdoc "Installs BB into a project"
    @moduledoc """
    #{@shortdoc}

    ## Example

    ```bash
    mix igniter.install bb
    ```
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter
    alias Igniter.Project.Module

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        composes: ["bb.add_robot"]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      robot_module = Module.module_name(igniter, "Robot")

      igniter
      |> Formatter.import_dep(:bb)
      |> Igniter.compose_task("bb.add_robot", ["--robot", inspect(robot_module)])
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
