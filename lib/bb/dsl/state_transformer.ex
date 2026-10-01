# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.StateTransformer do
  @moduledoc """
  Collects state definitions and injects state-related functions.

  This transformer:
  - Collects all states defined in the `states` section
  - Adds any of `BB.Dsl.State.built_in/0` the robot didn't define itself
  - Rejects any state shadowing `BB.Dsl.State.reserved_names/0`
  - Injects `__bb_states__/0` and `__bb_initial_state__/0` functions
  """
  use Spark.Dsl.Transformer

  alias BB.Dsl.State
  alias Spark.Dsl.{Extension, Transformer}
  alias Spark.Error.DslError

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
    user_states = user_states(dsl)

    case Enum.find(user_states, &(&1.name in State.reserved_names())) do
      nil -> inject_state_functions(dsl, collect_states(user_states))
      reserved -> {:error, reserved_state_error(dsl, reserved)}
    end
  end

  defp inject_state_functions(dsl, states) do
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

  defp reserved_state_error(dsl, state) do
    DslError.exception(
      module: Transformer.get_persisted(dsl, :module),
      path: [:states, state.name],
      message: """
      Cannot define a state named #{inspect(state.name)}.

      #{inspect(state.name)} belongs to the safety system: `BB.Safety` puts the
      robot there, `BB.Robot.Runtime.state/1` reports it ahead of the
      operational state machine, and no command runs while the robot is in it.
      A state of your own by that name could never be observed or acted on.

      Reserved: #{Enum.map_join(State.reserved_names(), ", ", &inspect/1)}
      """
    )
  end

  defp user_states(dsl) do
    dsl
    |> Transformer.get_entities([:states])
    |> Enum.filter(&is_struct(&1, State))
  end

  defp collect_states(user_states) do
    declared = Enum.map(user_states, & &1.name)

    Enum.reject(State.built_in(), &(&1.name in declared)) ++ user_states
  end

  defp get_initial_state(dsl) do
    Extension.get_opt(dsl, [:states], :initial_state, :idle)
  end
end
