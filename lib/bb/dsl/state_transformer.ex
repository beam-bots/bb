# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.StateTransformer do
  @moduledoc """
  Collects state definitions and injects state-related functions.

  This transformer:
  - Collects all states defined in the `states` section
  - Adds the built-in `:idle` state if not explicitly defined
  - Injects `__bb_states__/0` and `__bb_initial_state__/0` functions
  """
  use Spark.Dsl.Transformer

  alias BB.Dsl.State
  alias Spark.Dsl.{Extension, Transformer}

  @doc false
  @impl true
  def after?(BB.Dsl.DefaultNameTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(BB.Dsl.RobotTransformer), do: true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    states = collect_states(dsl)
    initial_state = get_initial_state(dsl)

    state_names = Enum.map(states, & &1.name)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @doc false
         def __bb_states__, do: unquote(Macro.escape(states))

         @doc false
         def __bb_state_names__, do: unquote(state_names)

         @doc false
         def __bb_initial_state__, do: unquote(initial_state)
       end
     )}
  end

  defp collect_states(dsl) do
    user_states =
      dsl
      |> Transformer.get_entities([:states])
      |> Enum.filter(&is_struct(&1, State))

    idle_defined? = Enum.any?(user_states, &(&1.name == :idle))

    if idle_defined? do
      user_states
    else
      idle_state = %State{
        name: :idle,
        doc: "Default idle state - robot is armed and ready for commands"
      }

      [idle_state | user_states]
    end
  end

  defp get_initial_state(dsl) do
    Extension.get_opt(dsl, [:states], :initial_state, :idle)
  end
end
