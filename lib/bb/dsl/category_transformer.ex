# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.CategoryTransformer do
  @moduledoc """
  Collects category definitions and injects category-related functions.

  This transformer:
  - Collects all categories defined in the `commands` section
  - Adds the built-in `:default` category if not explicitly defined
  - Injects `__bb_categories__/0` and `__bb_category_limits__/0` functions
  """
  use Spark.Dsl.Transformer

  alias BB.Dsl.Category
  alias Spark.Dsl.Transformer

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
    categories = collect_categories(dsl)
    category_limits = Map.new(categories, &{&1.name, &1.concurrency_limit})

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @doc false
         def __bb_categories__, do: unquote(Macro.escape(categories))

         @doc false
         def __bb_category_limits__, do: unquote(Macro.escape(category_limits))
       end
     )}
  end

  defp collect_categories(dsl) do
    user_categories =
      dsl
      |> Transformer.get_entities([:commands])
      |> Enum.filter(&is_struct(&1, Category))

    default_defined? = Enum.any?(user_categories, &(&1.name == :default))

    if default_defined? do
      user_categories
    else
      default_category = %Category{
        name: :default,
        doc: "Default category for commands without an explicit category",
        concurrency_limit: 1
      }

      [default_category | user_categories]
    end
  end
end
