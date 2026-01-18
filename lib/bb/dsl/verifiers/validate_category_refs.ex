# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidateCategoryRefs do
  @moduledoc """
  Validates that category references in commands are valid.

  This verifier checks that all commands referencing a category
  reference one that is defined in the `commands` section (or `:default`).
  """

  use Spark.Dsl.Verifier

  alias BB.Dsl.{Category, Command}
  alias Spark.Dsl.Verifier
  alias Spark.Error.DslError

  @impl true
  def verify(dsl_state) do
    module = Verifier.get_persisted(dsl_state, :module)
    categories = collect_category_names(dsl_state)

    with :ok <- verify_command_categories(dsl_state, categories, module) do
      verify_command_cancel(dsl_state, categories, module)
    end
  end

  defp collect_category_names(dsl_state) do
    user_categories =
      dsl_state
      |> Verifier.get_entities([:commands])
      |> Enum.filter(&is_struct(&1, Category))
      |> Enum.map(& &1.name)

    [:default | user_categories] |> Enum.uniq()
  end

  defp verify_command_categories(dsl_state, valid_categories, module) do
    dsl_state
    |> Verifier.get_entities([:commands])
    |> Enum.filter(&is_struct(&1, Command))
    |> Enum.reduce_while(:ok, fn command, :ok ->
      category = command.category

      cond do
        is_nil(category) ->
          {:cont, :ok}

        category in valid_categories ->
          {:cont, :ok}

        true ->
          {:halt,
           {:error,
            DslError.exception(
              module: module,
              path: [:commands, command.name, :category],
              message: """
              Command #{inspect(command.name)} references undefined category: #{inspect(category)}

              Valid categories: #{inspect(valid_categories)}

              Define the category in the commands section:

                  commands do
                    category #{inspect(category)}
                    # ...
                  end
              """
            )}}
      end
    end)
  end

  defp verify_command_cancel(dsl_state, valid_categories, module) do
    dsl_state
    |> Verifier.get_entities([:commands])
    |> Enum.filter(&is_struct(&1, Command))
    |> Enum.reduce_while(:ok, fn command, :ok ->
      invalid_categories = command.cancel -- valid_categories

      if invalid_categories == [] do
        {:cont, :ok}
      else
        {:halt,
         {:error,
          DslError.exception(
            module: module,
            path: [:commands, command.name, :cancel],
            message: """
            Command #{inspect(command.name)} cancel option references undefined categories: #{inspect(invalid_categories)}

            Valid categories: #{inspect(valid_categories)}

            Use :* to cancel all categories, or list specific categories.
            """
          )}}
      end
    end)
  end
end
