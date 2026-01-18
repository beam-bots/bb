# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidateStateRefs do
  @moduledoc """
  Validates that state references in commands are valid.

  This verifier checks:
  - All states in `allowed_states` are defined in the `states` section (or `:idle`)
  - The `initial_state` setting references a defined state
  - Commands using `{BB.Command.SetState, to: state}` reference valid states
  """

  use Spark.Dsl.Verifier

  alias BB.Dsl.{Command, State}
  alias Spark.Dsl.{Extension, Verifier}
  alias Spark.Error.DslError

  @impl true
  def verify(dsl_state) do
    module = Verifier.get_persisted(dsl_state, :module)
    states = collect_state_names(dsl_state)

    with :ok <- verify_initial_state(dsl_state, states, module),
         :ok <- verify_command_allowed_states(dsl_state, states, module) do
      verify_set_state_targets(dsl_state, states, module)
    end
  end

  defp collect_state_names(dsl_state) do
    user_states =
      dsl_state
      |> Verifier.get_entities([:states])
      |> Enum.filter(&is_struct(&1, State))
      |> Enum.map(& &1.name)

    # Include built-in states: :idle is always available as operational state,
    # :disarmed is a safety state that can be in allowed_states
    [:idle, :disarmed | user_states] |> Enum.uniq()
  end

  defp verify_initial_state(dsl_state, valid_states, module) do
    initial_state = Extension.get_opt(dsl_state, [:states], :initial_state, :idle)

    if initial_state in valid_states do
      :ok
    else
      {:error,
       DslError.exception(
         module: module,
         path: [:states, :initial_state],
         message: """
         Invalid initial_state: #{inspect(initial_state)}

         Valid states: #{inspect(valid_states)}
         """
       )}
    end
  end

  defp verify_command_allowed_states(dsl_state, valid_states, module) do
    dsl_state
    |> Verifier.get_entities([:commands])
    |> Enum.filter(&is_struct(&1, Command))
    |> Enum.reduce_while(:ok, fn command, :ok ->
      invalid_states = command.allowed_states -- (valid_states ++ [:executing])

      if invalid_states == [] do
        {:cont, :ok}
      else
        {:halt,
         {:error,
          DslError.exception(
            module: module,
            path: [:commands, command.name, :allowed_states],
            message: """
            Command #{inspect(command.name)} references undefined states: #{inspect(invalid_states)}

            Valid states: #{inspect(valid_states)}

            Note: :executing is always valid for preemption.
            """
          )}}
      end
    end)
  end

  defp verify_set_state_targets(dsl_state, valid_states, module) do
    dsl_state
    |> Verifier.get_entities([:commands])
    |> Enum.filter(&is_struct(&1, Command))
    |> Enum.reduce_while(:ok, fn command, :ok ->
      case command.handler do
        {BB.Command.SetState, opts} when is_list(opts) ->
          verify_set_state_target(command, opts, valid_states, module)

        _ ->
          {:cont, :ok}
      end
    end)
  end

  defp verify_set_state_target(command, opts, valid_states, module) do
    target = Keyword.get(opts, :to)

    cond do
      is_nil(target) ->
        {:halt,
         {:error,
          DslError.exception(
            module: module,
            path: [:commands, command.name, :handler],
            message: """
            BB.Command.SetState requires a :to option specifying the target state.

            Example: {BB.Command.SetState, to: :recording}
            """
          )}}

      target in valid_states ->
        {:cont, :ok}

      true ->
        {:halt,
         {:error,
          DslError.exception(
            module: module,
            path: [:commands, command.name, :handler],
            message: """
            BB.Command.SetState targets undefined state: #{inspect(target)}

            Valid states: #{inspect(valid_states)}
            """
          )}}
    end
  end
end
