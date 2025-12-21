# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.Schema do
  @moduledoc """
  Builds nested Spark.Options schemas from flat parameter definitions.

  This module converts the flat `[{path, opts}]` format from
  `__bb_parameter_schema__/0` into nested keyword lists suitable for
  validating `start_link` options.

  ## Example

      iex> flat = [
      ...>   {[:motion, :max_speed], [type: :float, default: 1.0]},
      ...>   {[:motion, :acceleration], [type: :float, default: 0.5]},
      ...>   {[:debug_mode], [type: :boolean, default: false]}
      ...> ]
      iex> BB.Parameter.Schema.build_nested_schema(flat)
      [
        debug_mode: [type: :boolean],
        motion: [type: :keyword_list, keys: [
          acceleration: [type: :float],
          max_speed: [type: :float]
        ]]
      ]
  """

  @doc """
  Builds a nested Spark.Options schema from flat `[{path, opts}]` list.

  The resulting schema can be passed to `Spark.Options.validate/2` to validate
  nested keyword lists like `[motion: [max_speed: 2.0]]`.

  All parameters are optional (no `:required` flag) since we're validating
  partial overrides. The `:default` key is removed from each parameter's opts
  since defaults are handled separately by the DSL.
  """
  @spec build_nested_schema([{[atom()], keyword()}]) :: keyword()
  def build_nested_schema(flat_schema) do
    flat_schema
    |> Enum.group_by(fn {[first | _], _opts} -> first end)
    |> Enum.map(fn {key, entries} -> build_entry(key, entries) end)
    |> Enum.sort_by(fn {key, _} -> key end)
  end

  defp build_entry(key, entries) do
    case entries do
      [{[^key], opts}] ->
        {key, clean_opts(opts)}

      _ ->
        nested_entries =
          Enum.map(entries, fn {[^key | rest], opts} -> {rest, opts} end)

        nested_keys = build_nested_keys(nested_entries)
        {key, [type: :keyword_list, keys: nested_keys]}
    end
  end

  defp build_nested_keys(entries) do
    entries
    |> Enum.group_by(fn {[first | _], _opts} -> first end)
    |> Enum.map(fn {key, group} -> build_entry(key, group) end)
    |> Enum.sort_by(fn {key, _} -> key end)
  end

  defp clean_opts(opts) do
    opts
    |> Keyword.delete(:default)
    |> Keyword.delete(:required)
  end

  @doc """
  Flattens nested keyword params to `[{path, value}]` format.

  This is the inverse of the nesting - it takes validated params like
  `[motion: [max_speed: 2.0]]` and returns `[{[:motion, :max_speed], 2.0}]`.
  """
  @spec flatten_params(keyword()) :: [{[atom()], term()}]
  def flatten_params(nested_params) do
    flatten_params(nested_params, [])
  end

  defp flatten_params(params, path_prefix) do
    Enum.flat_map(params, fn {key, value} ->
      path = path_prefix ++ [key]

      if Keyword.keyword?(value) and value != [] do
        flatten_params(value, path)
      else
        [{path, value}]
      end
    end)
  end
end
