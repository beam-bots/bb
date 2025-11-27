defmodule Kinetix.MixProject do
  use Mix.Project

  @moduledoc """
  The framework for resilient robotics.
  """

  @version "0.1.0"

  def project do
    [
      app: :kinetix,
      version: @version,
      description: @moduledoc,
      package: package(),
      docs: docs(),
      elixir: "~> 1.19",
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  defp package do
    [
      maintainers: ["James Harton <james@harton.nz>"],
      licenses: ["Apache-2.0"],
      links: %{
        "Source" => "https://harton.dev/kinetix/kinetix",
        "Sponsor" => "https://github.com/sponsors/jimsynz"
      }
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp docs do
    [
      main: "readme",
      extras: ["README.md"]
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:credo, "~> 1.7", only: [:dev, :test]},
      {:dialyxir, "~> 1.4", only: [:dev, :test]},
      {:ex_check, "~> 0.16", only: [:dev, :test]},
      {:ex_doc, ">= 0.0.0", only: [:dev, :test]},
      {:git_ops, "~> 2.9", only: [:dev, :test]},
      {:igniter, "~> 0.6", only: [:dev, :test]},
      {:mix_audit, "~> 2.1", only: [:dev, :test]}
    ]
  end
end
