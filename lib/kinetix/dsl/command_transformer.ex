# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.CommandTransformer do
  @moduledoc """
  Generates convenience functions for commands on the robot module.

  For each command defined in the DSL, this transformer generates a function
  on the robot module that calls `Kinetix.Robot.Runtime.execute/3`.

  ## Example

  Given a command definition:

      commands do
        command :navigate_to_pose do
          handler NavigateToPoseHandler
          argument :target_pose, Kinetix.Pose, required: true
          argument :tolerance, :float, default: 0.1
        end
      end

  This transformer generates:

      @spec navigate_to_pose(keyword()) :: {:ok, reference()} | {:error, term()}
      def navigate_to_pose(goal \\\\ []) do
        Kinetix.Robot.Runtime.execute(__MODULE__, :navigate_to_pose, Map.new(goal))
      end
  """
  use Spark.Dsl.Transformer
  alias Kinetix.Dsl.Info
  alias Kinetix.Robot.Runtime
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(Kinetix.Dsl.RobotTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    commands = Info.robot_commands(dsl)

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

      - `{:ok, result}` - Command succeeded with result
      - `{:ok, {:canceled, result}}` - Command was cancelled
      - `{:error, term()}` - Command failed or was rejected

      """
      @spec unquote(name)(keyword()) :: {:ok, term()} | {:error, term()}
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
