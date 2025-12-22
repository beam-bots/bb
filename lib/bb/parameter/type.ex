# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Type do
  @moduledoc """
  Validation for parameter type definitions in the DSL.

  Parameters can have simple types (`:float`, `:integer`, etc.) or unit types
  like `{:unit, :meter}`.
  """

  alias BB.Cldr.Unit

  @simple_types [:float, :integer, :boolean, :string, :atom]

  @doc """
  Validates a parameter type specification.

  Returns `{:ok, type}` for valid types or `{:error, message}` for invalid ones.

  ## Valid Types

  - Simple types: `:float`, `:integer`, `:boolean`, `:string`, `:atom`
  - Unit types: `{:unit, unit_type}` where `unit_type` is a valid CLDR unit

  ## Examples

      iex> BB.Parameter.Type.validate(:float)
      {:ok, :float}

      iex> BB.Parameter.Type.validate({:unit, :meter})
      {:ok, {:unit, :meter}}

      iex> BB.Parameter.Type.validate(:invalid)
      {:error, "Expected one of [:float, :integer, :boolean, :string, :atom] or {:unit, unit_type}, got: :invalid"}
  """
  @spec validate(term()) :: {:ok, atom() | {:unit, atom()}} | {:error, String.t()}
  def validate(type) when type in @simple_types, do: {:ok, type}

  def validate({:unit, unit_type}) when is_atom(unit_type) do
    case Unit.validate_unit(unit_type) do
      {:ok, _, _} -> {:ok, {:unit, unit_type}}
      {:error, _} -> {:error, "Invalid unit type: #{inspect(unit_type)}"}
    end
  end

  def validate(other) do
    {:error,
     "Expected one of #{inspect(@simple_types)} or {:unit, unit_type}, got: #{inspect(other)}"}
  end
end
