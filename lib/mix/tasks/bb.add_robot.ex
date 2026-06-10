# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Bb.AddRobot do
    @shortdoc "Adds a new robot module to your project"
    @moduledoc """
    #{@shortdoc}

    ## Example

    ```bash
    mix bb.add_robot --robot MyApp.Robots.MainRobot
    ```

    ## Options

    * `--robot` - The module name for the robot (defaults to {AppPrefix}.Robot)
    """

    use Igniter.Mix.Task

    alias Igniter.Code.{Common, Function}
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
      robot_module = BB.Igniter.robot_module(igniter)

      igniter
      |> create_robot_module(robot_module)
      |> add_to_supervision_tree(robot_module)
    end

    defp create_robot_module(igniter, module) do
      case Module.module_exists(igniter, module) do
        {true, igniter} ->
          igniter

        {false, igniter} ->
          Module.create_module(igniter, module, robot_module_contents())
      end
    end

    defp robot_module_contents do
      """
      use BB

      commands do
        command :arm do
          handler(BB.Command.Arm)
          allowed_states([:disarmed])
        end

        command :disarm do
          handler(BB.Command.Disarm)
          allowed_states([:idle])
        end
      end

      topology do
        link :base_link do
        end
      end
      """
    end

    # The robot starts with the application. Its child-spec opts come from a
    # generated `robot_opts/0` helper so a project can boot into `:kinematic`
    # simulation with `SIMULATE=1 iex -S mix`, or feed startup opts from config,
    # without editing the supervision tree by hand.
    defp add_to_supervision_tree(igniter, robot_module) do
      igniter
      |> add_robot_child(robot_module)
      |> set_robot_child_opts(robot_module)
      |> add_robot_opts_function(robot_module)
    end

    defp add_robot_child(igniter, robot_module) do
      Igniter.Project.Application.add_new_child(igniter, {robot_module, []},
        after: fn _ -> true end
      )
    end

    # `opts_updater` only runs against an already-present child, so this second
    # call replaces the `[]` placed by `add_robot_child/2` with `robot_opts()`.
    defp set_robot_child_opts(igniter, robot_module) do
      Igniter.Project.Application.add_new_child(igniter, {robot_module, []},
        opts_updater: fn zipper ->
          {:ok, Sourceror.Zipper.replace(zipper, quote(do: robot_opts()))}
        end
      )
    end

    defp add_robot_opts_function(igniter, robot_module) do
      app_module = application_module(igniter)
      app_name = Igniter.Project.Application.app_name(igniter)
      code = robot_opts_function_code(app_name, robot_module)

      Module.find_and_update_module!(igniter, app_module, fn zipper ->
        if robot_opts_defined?(zipper) do
          {:ok, zipper}
        else
          {:ok, Common.add_code(zipper, code)}
        end
      end)
    end

    defp robot_opts_defined?(zipper) do
      with :error <- Function.move_to_def(zipper, :robot_opts, 0),
           :error <- Function.move_to_defp(zipper, :robot_opts, 0) do
        false
      else
        {:ok, _} -> true
      end
    end

    defp application_module(igniter) do
      Elixir.Module.concat(Module.module_name_prefix(igniter), Application)
    end

    defp robot_opts_function_code(app_name, robot_module) do
      """
      defp robot_opts do
        if System.get_env("SIMULATE") do
          [simulation: :kinematic]
        else
          Application.get_env(#{inspect(app_name)}, #{inspect(robot_module)}, [])
        end
      end
      """
    end
  end
else
  defmodule Mix.Tasks.Bb.AddRobot do
    @shortdoc "Adds a new robot module to your project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb.add_robot task requires igniter.

          mix deps.get
          mix bb.add_robot --robot MyApp.Robot
      """)

      exit({:shutdown, 1})
    end
  end
end
