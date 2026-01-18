# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.WildcardExpansionTransformer do
  @moduledoc """
  Expands `:*` wildcards in command `allowed_states` and `cancel` options.

  This transformer:
  - Expands `:*` in `allowed_states` to all defined states (including `:idle`, `:disarmed`)
  - Expands `:*` in `cancel` to all defined categories (including `:default`)
  - Runs after `StateTransformer` and `CategoryTransformer` so state/category lists are available
  """
  use Spark.Dsl.Transformer

  alias BB.Dsl.{Category, Command, State}
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(BB.Dsl.StateTransformer), do: true
  def after?(BB.Dsl.CategoryTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(BB.Dsl.RobotTransformer), do: true
  def before?(BB.Dsl.CommandTransformer), do: true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    all_states = collect_state_names(dsl)
    all_categories = collect_category_names(dsl)

    commands =
      dsl
      |> Transformer.get_entities([:commands])
      |> Enum.filter(&is_struct(&1, Command))

    if Enum.empty?(commands) do
      {:ok, dsl}
    else
      expand_commands(dsl, commands, all_states, all_categories)
    end
  end

  defp expand_commands(dsl, commands, all_states, all_categories) do
    Enum.reduce(commands, {:ok, dsl}, fn command, {:ok, dsl_acc} ->
      expanded_command =
        command
        |> expand_allowed_states(all_states)
        |> expand_cancel(all_categories)

      {:ok, Transformer.replace_entity(dsl_acc, [:commands], expanded_command)}
    end)
  end

  defp expand_allowed_states(command, all_states) do
    expanded =
      command.allowed_states
      |> Enum.flat_map(fn
        :* -> all_states
        state -> [state]
      end)
      |> Enum.uniq()

    %{command | allowed_states: expanded}
  end

  defp expand_cancel(command, all_categories) do
    expanded =
      command.cancel
      |> Enum.flat_map(fn
        :* -> all_categories
        category -> [category]
      end)
      |> Enum.uniq()

    %{command | cancel: expanded}
  end

  defp collect_state_names(dsl) do
    user_states =
      dsl
      |> Transformer.get_entities([:states])
      |> Enum.filter(&is_struct(&1, State))
      |> Enum.map(& &1.name)

    # Include built-in states
    [:idle, :disarmed | user_states] |> Enum.uniq()
  end

  defp collect_category_names(dsl) do
    user_categories =
      dsl
      |> Transformer.get_entities([:commands])
      |> Enum.filter(&is_struct(&1, Category))
      |> Enum.map(& &1.name)

    [:default | user_categories] |> Enum.uniq()
  end
end
