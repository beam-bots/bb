# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Bb.VerifyDocs do
  @moduledoc """
  Compiles every robot definition embedded in a set of markdown files.

  A documented robot that no longer compiles is worse than no example, and
  nothing else in the build ever reads the docs — so this reads them.

      mix bb.verify_docs                       # this package's docs
      mix bb.verify_docs ../bb_servo_pigpio    # somebody else's

  Given directories, every `.md` beneath them is searched. Only fenced
  `elixir` blocks defining a module with `use BB` are considered, and only the
  `defmodule … end` spans within them are compiled — a block that also shows
  calls against the robot won't have those calls executed.

  Examples referencing modules this package doesn't depend on (a servo driver,
  say) still compile, but Spark can't run its behaviour and options checks over
  them and says so on stderr. Those examples get the DSL's structural checks
  only, which is still enough to catch a malformed topology.
  """
  @shortdoc "Compile the robot definitions embedded in the documentation"

  use Mix.Task

  @default_paths ["README.md", "documentation", "usage-rules"]

  @impl Mix.Task
  def run(args) do
    Mix.Task.run("app.start")

    args
    |> case do
      [] -> @default_paths
      paths -> paths
    end
    |> Enum.flat_map(&expand/1)
    |> verify()
  end

  defp expand(path) do
    cond do
      File.dir?(path) -> path |> Path.join("**/*.md") |> Path.wildcard()
      File.exists?(path) -> [path]
      true -> []
    end
  end

  defp verify(paths) do
    results = Enum.flat_map(paths, &check_file/1)
    {ok, bad} = Enum.split_with(results, &match?({:ok, _, _}, &1))

    Enum.each(bad, fn {:error, path, line, message} ->
      Mix.shell().error("\nFAIL #{path}:#{line}\n#{message}")
    end)

    Mix.shell().info("\n#{length(ok)} robot definitions compiled, #{length(bad)} failed")

    if bad != [] do
      Mix.raise("#{length(bad)} documented robot definition(s) failed to compile")
    end
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
      {"```elixir", line}, {blocks, nil, _} ->
        {blocks, line, []}

      {"```", _}, {blocks, start, acc} when is_integer(start) ->
        {[{start, acc |> Enum.reverse() |> Enum.join("\n")} | blocks], nil, []}

      {text, _}, {blocks, start, acc} when is_integer(start) ->
        {blocks, start, [text | acc]}

      _, state ->
        state
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
