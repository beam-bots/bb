# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Bb.AddNxBackendTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  describe "--backend exla" do
    test "adds the exla dep" do
      test_project()
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])
      |> assert_has_patch("mix.exs", """
      + |      {:exla, "~> 0.10"}
      """)
    end

    test "writes the default_backend config to runtime.exs" do
      igniter =
        test_project()
        |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])

      {_, source} = Rewrite.source(igniter.rewrite, "config/runtime.exs")
      assert source.content =~ "config :nx, default_backend: EXLA.Backend"
    end

    test "does not write to config.exs (compile-time backend would crash BB transformers)" do
      igniter =
        test_project()
        |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])

      case Rewrite.source(igniter.rewrite, "config/config.exs") do
        {:ok, source} -> refute source.content =~ "default_backend"
        {:error, _} -> :ok
      end
    end
  end

  describe "--backend torchx" do
    test "adds the torchx dep" do
      test_project()
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "torchx"])
      |> assert_has_patch("mix.exs", """
      + |      {:torchx, "~> 0.10"}
      """)
    end

    test "writes the default_backend config to runtime.exs" do
      igniter =
        test_project()
        |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "torchx"])

      {_, source} = Rewrite.source(igniter.rewrite, "config/runtime.exs")
      assert source.content =~ "config :nx, default_backend: Torchx.Backend"
    end
  end

  describe "--backend binary" do
    test "does not add any dep or config" do
      test_project()
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "binary"])
      |> assert_unchanged()
    end
  end

  describe "non-interactive default" do
    test "with --yes defaults to exla" do
      test_project()
      |> Igniter.compose_task("bb.add_nx_backend", ["--yes"])
      |> assert_has_patch("mix.exs", """
      + |      {:exla, "~> 0.10"}
      """)
    end
  end

  describe "skipping" do
    test "skips when :nx is already configured in runtime.exs" do
      test_project(
        files: %{
          "config/runtime.exs" => """
          import Config

          config :nx, default_backend: Some.Other.Backend
          """
        }
      )
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])
      |> assert_unchanged()
    end

    test "skips when :nx is already configured in config.exs" do
      test_project(
        files: %{
          "config/config.exs" => """
          import Config

          config :nx, default_backend: Some.Other.Backend
          """
        }
      )
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])
      |> assert_unchanged()
    end

    test "skips when :exla is already a dep" do
      project = test_project()

      mix_exs =
        project.rewrite
        |> Rewrite.source!("mix.exs")
        |> Rewrite.Source.get(:content)
        |> String.replace(
          ~r/(defp deps do\s*\n\s*\[)/,
          "\\1\n      {:exla, \"~> 0.10\"},"
        )

      test_project(files: %{"mix.exs" => mix_exs})
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "torchx"])
      |> assert_unchanged()
    end

    test "skips when :torchx is already a dep" do
      project = test_project()

      mix_exs =
        project.rewrite
        |> Rewrite.source!("mix.exs")
        |> Rewrite.Source.get(:content)
        |> String.replace(
          ~r/(defp deps do\s*\n\s*\[)/,
          "\\1\n      {:torchx, \"~> 0.10\"},"
        )

      test_project(files: %{"mix.exs" => mix_exs})
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])
      |> assert_unchanged()
    end
  end

  describe "idempotency" do
    test "running twice does not duplicate the dep or config" do
      test_project()
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])
      |> apply_igniter!()
      |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "exla"])
      |> assert_unchanged()
    end
  end

  describe "invalid input" do
    test "raises on unknown backend value" do
      assert_raise Mix.Error, ~r/Unknown Nx backend/, fn ->
        test_project()
        |> Igniter.compose_task("bb.add_nx_backend", ["--backend", "tensorflow"])
      end
    end
  end
end
