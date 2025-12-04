# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Kinetix.AddRobot do
    @shortdoc "Adds a new robot module to your project"
    @moduledoc """
    #{@shortdoc}

    ## Example

    ```bash
    mix kinetix.add_robot --robot MyApp.Robots.MainRobot
    ```

    ## Options

    * `--robot` - The module name for the robot (defaults to {AppPrefix}.Robot)
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Application
    alias Igniter.Project.Module

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [robot: :string],
        aliases: [r: :robot]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      options = igniter.args.options

      robot_module =
        case Keyword.get(options, :robot) do
          nil -> Module.module_name(igniter, "Robot")
          name -> Module.parse(name)
        end

      igniter
      |> create_robot_module(robot_module)
      |> add_to_supervision_tree(robot_module)
    end

    defp create_robot_module(igniter, module) do
      contents = """
      use Kinetix

      commands do
        command :arm do
          handler(Kinetix.Command.Arm)
          allowed_states([:disarmed])
        end

        command :disarm do
          handler(Kinetix.Command.Disarm)
          allowed_states([:idle])
        end
      end

      topology do
        link :base_link do
        end
      end
      """

      Module.create_module(igniter, module, contents)
    end

    defp add_to_supervision_tree(igniter, robot_module) do
      Application.add_new_child(
        igniter,
        {robot_module, []},
        after: fn _ -> true end
      )
    end
  end
else
  defmodule Mix.Tasks.Kinetix.AddRobot do
    @shortdoc "Adds a new robot module to your project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The kinetix.add_robot task requires igniter.

          mix deps.get
          mix kinetix.add_robot --robot MyApp.Robot
      """)

      exit({:shutdown, 1})
    end
  end
end
