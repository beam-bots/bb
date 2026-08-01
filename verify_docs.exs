# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

# Extracts every ```elixir block containing `use BB` from the given markdown
# files and compiles it, so a robot definition in the docs can't silently rot.
#
#     mix run verify_docs.exs ../bb_servo_pca9685/documentation/tutorials/*.md

defmodule DocVerifier do
  def run(paths) do
    results = Enum.flat_map(paths, &check_file/1)
    {ok, bad} = Enum.split_with(results, &match?({:ok, _, _}, &1))

    Enum.each(bad, fn {:error, path, line, message} ->
      IO.puts("\nFAIL #{path}:#{line}\n#{message}")
    end)

    IO.puts("\n#{length(ok)} robot definitions compiled, #{length(bad)} failed")
    if bad != [], do: System.halt(1)
  end

  defp check_file(path) do
    path
    |> File.read!()
    |> extract_blocks()
    |> Enum.map(fn {line, code} -> compile(path, line, code) end)
  end

  # Only blocks that define a module using BB — snippets of bare DSL fragments
  # can't compile standalone and aren't claiming to.
  defp extract_blocks(source) do
    source
    |> String.split("\n")
    |> Enum.with_index(1)
    |> Enum.reduce({[], nil, []}, fn
      {"```elixir", line}, {blocks, nil, _} -> {blocks, line, []}
      {"```", _}, {blocks, start, acc} when is_integer(start) ->
        {[{start, acc |> Enum.reverse() |> Enum.join("\n")} | blocks], nil, []}
      {text, _}, {blocks, start, acc} when is_integer(start) -> {blocks, start, [text | acc]}
      _, state -> state
    end)
    |> elem(0)
    |> Enum.reverse()
    |> Enum.filter(fn {_line, code} ->
      Regex.match?(~r/^\s*use BB(\.Robot)?\s*$/m, code) and String.contains?(code, "defmodule")
    end)
  end

  # A block often shows a module *and* then calls against it. `Code.compile_string`
  # would execute those calls against a robot that isn't running, so keep only the
  # top-level `defmodule … end` spans — the part that has to compile.
  defp module_definitions(code) do
    code
    |> String.split("\n")
    |> Enum.reduce({[], false}, fn
      line, {acc, false} ->
        if String.starts_with?(line, "defmodule "), do: {[line | acc], true}, else: {acc, false}

      line, {acc, true} ->
        {[line | acc], line != "end"}
    end)
    |> elem(0)
    |> Enum.reverse()
    |> Enum.join("\n")
  end

  defp compile(path, line, code) do
    code = module_definitions(code)

    # Each block gets a unique module namespace so repeated `MyRobot` examples
    # across files don't collide.
    suffix = "_DocCheck#{:erlang.phash2({path, line})}"
    code = Regex.replace(~r/defmodule ([A-Z][A-Za-z0-9_.]*)/, code, "defmodule \\1#{suffix}")

    try do
      Code.compile_string(code, path)
      {:ok, path, line}
    rescue
      e -> {:error, path, line, Exception.message(e)}
    end
  end
end

DocVerifier.run(System.argv())
