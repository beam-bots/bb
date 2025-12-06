# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.CommandTransformer do
  @moduledoc """
  Generates convenience functions for commands on the robot module.

  For each command defined in the DSL, this transformer generates a function
  on the robot module that calls `BB.Robot.Runtime.execute/3`.

  ## Example

  Given a command definition:

      commands do
        command :navigate_to_pose do
          handler NavigateToPoseHandler
          argument :target_pose, BB.Pose, required: true
          argument :tolerance, :float, default: 0.1
        end
      end

  This transformer generates:

      @spec navigate_to_pose(keyword()) :: {:ok, Task.t()} | {:error, term()}
      def navigate_to_pose(goal \\\\ []) do
        BB.Robot.Runtime.execute(__MODULE__, :navigate_to_pose, Map.new(goal))
      end

  The caller can then await the task to get the result:

      {:ok, task} = MyRobot.navigate_to_pose(target_pose: pose)
      {:ok, result} = Task.await(task)
  """
  use Spark.Dsl.Transformer
  alias BB.Dsl.Info
  alias BB.Robot.Runtime
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(BB.Dsl.RobotTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    commands = Info.commands(dsl)

    if Enum.empty?(commands) do
      {:ok, dsl}
    else
      inject_command_functions(dsl, commands)
    end
  end

  defp inject_command_functions(dsl, commands) do
    functions = Enum.map(commands, &generate_function/1)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         (unquote_splicing(functions))
       end
     )}
  end

  defp generate_function(command) do
    name = command.name
    args_doc = build_args_doc(command.arguments)

    quote do
      @doc """
      Execute the `#{unquote(name)}` command.
      #{unquote(args_doc)}
      ## Returns

      - `{:ok, Task.t()}` - Command started, await the task for the result
      - `{:error, term()}` - Command could not be started

      ## Example

          {:ok, task} = #{unquote(name)}(goal_args)
          {:ok, result} = Task.await(task)

      """
      @spec unquote(name)(keyword()) :: {:ok, Task.t()} | {:error, term()}
      def unquote(name)(goal \\ []) do
        unquote(Runtime).execute(__MODULE__, unquote(name), Map.new(goal))
      end
    end
  end

  defp build_args_doc([]), do: ""

  defp build_args_doc(arguments) do
    args_list =
      Enum.map_join(arguments, "\n", fn arg ->
        required = if arg.required, do: " (required)", else: ""
        default = if arg.default != nil, do: ", default: `#{inspect(arg.default)}`", else: ""
        doc = if arg.doc, do: " - #{arg.doc}", else: ""
        "- `#{arg.name}`: `#{inspect(arg.type)}`#{required}#{default}#{doc}"
      end)

    """

    ## Arguments

    #{args_list}
    """
  end
end
