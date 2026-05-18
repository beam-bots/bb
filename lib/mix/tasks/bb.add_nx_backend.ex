# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Bb.AddNxBackend do
    @shortdoc "Configures an Nx backend for the project"
    @moduledoc """
    #{@shortdoc}

    BB uses Nx for kinematics and motion planning. Nx defaults to
    `Nx.BinaryBackend` — a pure-Elixir reference implementation that is fine
    for sanity checks but orders of magnitude slower than the native backends.
    This task prompts the user to pick a backend and applies the dep + config
    changes.

    The `config :nx, default_backend: …` line is written to `config/runtime.exs`,
    not `config/config.exs`. BB performs compile-time tensor operations in
    its Spark transformer chain, and EXLA/Torchx aren't started during
    compilation — so a compile-time default crashes the robot module compile.

    Skips itself entirely if `:nx` is already configured in `runtime.exs` or
    `config.exs`, or if `:exla` / `:torchx` is already declared in `mix.exs`.

    ## Examples

    ```bash
    # Interactive (prompts to choose a backend)
    mix bb.add_nx_backend

    # Non-interactive (defaults to exla)
    mix bb.add_nx_backend --yes

    # Explicit choice
    mix bb.add_nx_backend --backend exla
    mix bb.add_nx_backend --backend torchx
    mix bb.add_nx_backend --backend binary
    ```

    ## Options

    * `--backend` - One of `exla`, `torchx`, or `binary`. Skips the prompt.
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Config
    alias Igniter.Project.Deps
    alias Igniter.Util.IO, as: IgniterIO

    @nx_version "~> 0.10"

    @backends [
      {"EXLA (recommended — JIT-compiled native CPU/GPU)", :exla},
      {"Torchx (libtorch — useful if you'll integrate with PyTorch models)", :torchx},
      {"Binary (pure Elixir, slow, no extra dependencies)", :binary}
    ]

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [backend: :string],
        aliases: [b: :backend]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      cond do
        already_configured?(igniter) ->
          igniter

        backend = explicit_backend(igniter) ->
          apply_backend(igniter, backend)

        non_interactive?(igniter) ->
          apply_backend(igniter, :exla)

        true ->
          apply_backend(igniter, prompt_backend())
      end
    end

    defp non_interactive?(igniter) do
      igniter.args.options[:yes] || igniter.assigns[:test_mode?]
    end

    defp already_configured?(igniter) do
      Config.configures_root_key?(igniter, "runtime.exs", :nx) or
        Config.configures_root_key?(igniter, "config.exs", :nx) or
        has_dep?(igniter, :exla) or
        has_dep?(igniter, :torchx)
    end

    defp has_dep?(igniter, name) do
      case Deps.get_dep(igniter, name) do
        {:ok, nil} -> false
        {:ok, _} -> true
        {:error, _} -> false
      end
    end

    defp explicit_backend(igniter) do
      case Keyword.get(igniter.args.options, :backend) do
        nil -> nil
        value -> parse_backend(value)
      end
    end

    defp parse_backend(value) do
      case String.downcase(to_string(value)) do
        "exla" ->
          :exla

        "torchx" ->
          :torchx

        "binary" ->
          :binary

        other ->
          Mix.raise("Unknown Nx backend: #{inspect(other)}. Expected exla, torchx, or binary.")
      end
    end

    defp prompt_backend do
      IgniterIO.select(
        "Which Nx backend would you like to use?",
        @backends,
        display: &elem(&1, 0),
        default: List.first(@backends)
      )
      |> elem(1)
    end

    defp apply_backend(igniter, :exla) do
      igniter
      |> Deps.add_dep({:exla, @nx_version}, on_exists: :skip)
      |> set_default_backend("EXLA.Backend")
    end

    defp apply_backend(igniter, :torchx) do
      igniter
      |> Deps.add_dep({:torchx, @nx_version}, on_exists: :skip)
      |> set_default_backend("Torchx.Backend")
    end

    defp apply_backend(igniter, :binary) do
      Igniter.add_notice(
        igniter,
        "Nx will use the default BinaryBackend (pure Elixir). Performance will be limited; switch to EXLA or Torchx for serious workloads."
      )
    end

    # Writes `config :nx, default_backend: <Backend>` to `config/runtime.exs`,
    # not `config/config.exs`. BB performs compile-time tensor operations in
    # its Spark transformer chain (BB.Robot.Builder); the EXLA/Torchx backend
    # processes aren't started during compilation, so a compile-time default
    # crashes the user's robot-module compile with a `no process: EXLA.Client`
    # error.
    defp set_default_backend(igniter, backend_module) do
      Config.configure(
        igniter,
        "runtime.exs",
        :nx,
        [:default_backend],
        {:code, Sourceror.parse_string!(backend_module)}
      )
    end
  end
else
  defmodule Mix.Tasks.Bb.AddNxBackend do
    @shortdoc "Configures an Nx backend for the project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb.add_nx_backend task requires igniter.

          mix deps.get
          mix bb.add_nx_backend
      """)

      exit({:shutdown, 1})
    end
  end
end
