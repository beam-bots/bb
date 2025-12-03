# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.DefaultNameTransformer do
  @moduledoc "Ensures that the default robot name is present"
  use Spark.Dsl.Transformer
  alias Kinetix.Dsl.Info
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: true

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    case Info.robot_name(dsl) do
      {:ok, _} ->
        {:ok, dsl}

      :error ->
        name =
          module
          |> Module.split()
          |> List.last()
          |> Macro.underscore()
          |> String.to_atom()

        {:ok, Transformer.set_option(dsl, [:robot], :name, name)}
    end
  end
end
