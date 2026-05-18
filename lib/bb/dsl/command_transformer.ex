# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.CommandTransformer do
  @moduledoc """
  Generates convenience functions for commands on the robot module and
  resolves arm/disarm command routing.

  For each command defined in the DSL, this transformer generates a function
  on the robot module that calls `BB.Robot.Runtime.execute/3`.

  It also:

  - Sets `arm: true` implicitly on commands whose handler is `BB.Command.Arm`,
    and `disarm: true` implicitly on commands using `BB.Command.Disarm`. This
    preserves the historical behaviour of `BB.Safety.arm/1`/`BB.Safety.disarm/2`
    for robots that use the built-in command modules: those calls now dispatch
    via the runtime instead of flipping safety state directly, while still
    producing the same observable outcome.
  - Validates that at most one command in the DSL is `arm`-flagged and at most
    one is `disarm`-flagged, that no single command sets both, and that the
    `allowed_states` of flagged commands are compatible with the state(s) the
    command is supposed to transition out of.
  - Injects `__bb_arm_command__/0` and `__bb_disarm_command__/0` lookup
    functions on the robot module, returning the name (atom) of the
    flagged command or `nil`. `BB.Safety` uses these to decide whether to
    route through the command pipeline or fall through to the safety
    controller's direct state-flip behaviour.

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

      @spec navigate_to_pose(keyword()) :: {:ok, pid()} | {:error, term()}
      def navigate_to_pose(goal \\\\ []) do
        BB.Robot.Runtime.execute(__MODULE__, :navigate_to_pose, Map.new(goal))
      end

  The caller can then await the command to get the result:

      {:ok, cmd} = MyRobot.navigate_to_pose(target_pose: pose)
      {:ok, result} = BB.Command.await(cmd)
  """
  use Spark.Dsl.Transformer
  alias BB.Dsl.{Command, Info}
  alias BB.Robot.Runtime
  alias Spark.Dsl.Transformer
  alias Spark.Error.DslError

  @arm_handler BB.Command.Arm
  @disarm_handler BB.Command.Disarm

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
    commands =
      dsl
      |> Info.commands()
      |> Enum.filter(&is_struct(&1, Command))

    commands = Enum.map(commands, &apply_implicit_flags/1)

    with :ok <- validate_flags(commands, dsl) do
      dsl = put_commands(dsl, commands)
      arm_command = find_flagged(commands, :arm)
      disarm_command = find_flagged(commands, :disarm)

      dsl =
        Transformer.eval(
          dsl,
          [arm_command: arm_command, disarm_command: disarm_command],
          quote do
            @doc false
            def __bb_arm_command__, do: unquote(arm_command)

            @doc false
            def __bb_disarm_command__, do: unquote(disarm_command)
          end
        )

      if commands == [] do
        {:ok, dsl}
      else
        inject_command_functions(dsl, commands)
      end
    end
  end

  defp apply_implicit_flags(%Command{} = command) do
    handler_module = handler_module(command.handler)

    command
    |> maybe_set_flag(:arm, handler_module == @arm_handler)
    |> maybe_set_flag(:disarm, handler_module == @disarm_handler)
  end

  defp maybe_set_flag(%Command{} = command, _key, false), do: command

  defp maybe_set_flag(%Command{} = command, key, true) do
    case Map.fetch!(command, key) do
      true -> command
      false -> Map.put(command, key, true)
    end
  end

  defp handler_module({module, _opts}) when is_atom(module), do: module
  defp handler_module(module) when is_atom(module), do: module
  defp handler_module(_), do: nil

  defp find_flagged(commands, key) do
    case Enum.find(commands, &Map.fetch!(&1, key)) do
      nil -> nil
      command -> command.name
    end
  end

  defp validate_flags(commands, dsl) do
    with :ok <- validate_unique_flag(commands, :arm, dsl),
         :ok <- validate_unique_flag(commands, :disarm, dsl),
         :ok <- validate_not_both(commands, dsl),
         :ok <- validate_arm_states(commands, dsl) do
      validate_disarm_states(commands, dsl)
    end
  end

  defp validate_unique_flag(commands, key, dsl) do
    flagged = Enum.filter(commands, &Map.fetch!(&1, key))

    case flagged do
      [] ->
        :ok

      [_one] ->
        :ok

      many ->
        names = Enum.map(many, & &1.name)

        {:error,
         DslError.exception(
           module: Transformer.get_persisted(dsl, :module),
           path: [:commands],
           message:
             "Multiple commands have `#{key} true` set: #{inspect(names)}. " <>
               "Only one command per robot may be the canonical " <>
               "#{key_to_phrase(key)} command."
         )}
    end
  end

  defp validate_not_both(commands, dsl) do
    both = Enum.filter(commands, &(&1.arm and &1.disarm))

    case both do
      [] ->
        :ok

      [command | _] ->
        {:error,
         DslError.exception(
           module: Transformer.get_persisted(dsl, :module),
           path: [:commands, command.name],
           message:
             "Command #{inspect(command.name)} has both `arm true` and " <>
               "`disarm true`. These flags are mutually exclusive — a single " <>
               "command cannot be both the arming and the disarming command."
         )}
    end
  end

  defp validate_arm_states(commands, dsl) do
    case find_command_with_flag(commands, :arm) do
      nil ->
        :ok

      command ->
        if :disarmed in command.allowed_states do
          :ok
        else
          {:error,
           DslError.exception(
             module: Transformer.get_persisted(dsl, :module),
             path: [:commands, command.name],
             message:
               "Arm-flagged command #{inspect(command.name)} must include " <>
                 "`:disarmed` in its `allowed_states` (got " <>
                 "#{inspect(command.allowed_states)}). An arming command must " <>
                 "be runnable while the robot is disarmed."
           )}
        end
    end
  end

  defp validate_disarm_states(commands, dsl) do
    case find_command_with_flag(commands, :disarm) do
      nil ->
        :ok

      command ->
        if armed_state_reachable?(command.allowed_states) do
          :ok
        else
          {:error,
           DslError.exception(
             module: Transformer.get_persisted(dsl, :module),
             path: [:commands, command.name],
             message:
               "Disarm-flagged command #{inspect(command.name)} must be " <>
                 "runnable from an armed state (got `allowed_states: " <>
                 "#{inspect(command.allowed_states)}`). Include `:idle` (or " <>
                 "another non-`:disarmed`, non-`:error` state) so the command " <>
                 "is reachable when the robot is armed."
           )}
        end
    end
  end

  defp find_command_with_flag(commands, key) do
    Enum.find(commands, &Map.fetch!(&1, key))
  end

  # An armed state is anything other than :disarmed/:disarming/:error.
  # The wildcard `:*` is also acceptable.
  defp armed_state_reachable?(allowed_states) do
    Enum.any?(allowed_states, fn state ->
      state == :* or state not in [:disarmed, :disarming, :error]
    end)
  end

  defp key_to_phrase(:arm), do: "arming"
  defp key_to_phrase(:disarm), do: "disarming"

  defp put_commands(dsl, commands) do
    Enum.reduce(commands, dsl, fn command, dsl ->
      Transformer.replace_entity(dsl, [:commands], command)
    end)
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

      - `{:ok, pid()}` - Command started, use `BB.Command.await/2` for the result
      - `{:error, term()}` - Command could not be started

      ## Example

          {:ok, cmd} = #{unquote(name)}(goal_args)
          {:ok, result} = BB.Command.await(cmd)

      """
      @spec unquote(name)(keyword()) :: {:ok, pid()} | {:error, term()}
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
