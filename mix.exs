# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.MixProject do
  use Mix.Project

  @moduledoc """
  Beam Bots - The framework for resilient robotics.
  """

  @version "0.13.2"

  def project do
    [
      aliases: aliases(),
      app: :bb,
      consolidate_protocols: Mix.env() == :prod,
      deps: deps(),
      description: @moduledoc,
      dialyzer: dialyzer(),
      docs: docs(),
      elixir: "~> 1.19",
      elixirc_paths: elixirc_paths(Mix.env()),
      package: package(),
      start_permanent: Mix.env() == :prod,
      version: @version
    ]
  end

  defp dialyzer do
    [
      plt_add_apps: [:mix]
    ]
  end

  defp package do
    [
      maintainers: ["James Harton <james@harton.nz>"],
      licenses: ["Apache-2.0"],
      links: %{
        "Source" => "https://github.com/beam-bots/bb",
        "Sponsor" => "https://github.com/sponsors/jimsynz"
      }
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger, :xmerl],
      mod: {BB.Application, []}
    ]
  end

  defp docs do
    [
      main: "readme",
      extras:
        ["README.md", "CHANGELOG.md"]
        |> Enum.concat(Path.wildcard("documentation/**/*.{md,livemd,cheatmd}")),
      groups_for_extras: [
        Tutorials: ~r/tutorials\//,
        Topics: ~r/topics\//,
        "DSL Reference": ~r/dsls\//
      ],
      groups_for_modules: [
        Core: [
          BB,
          BB.Robot,
          BB.Supervisor,
          BB.PubSub,
          BB.Telemetry
        ],
        DSL: [
          BB.Dsl,
          ~r/^BB\.Dsl\./
        ],
        Commands: [
          BB.Command,
          ~r/^BB\.Command\./
        ],
        Controllers: [
          BB.Controller,
          ~r/^BB\.Controller\./
        ],
        Sensors: [
          BB.Sensor,
          ~r/^BB\.Sensor\./
        ],
        Actuators: [
          BB.Actuator,
          ~r/^BB\.Actuator\./
        ],
        Messages: [
          BB.Message,
          ~r/^BB\.Message\./
        ],
        Safety: [
          BB.Safety,
          ~r/^BB\.Safety\./
        ],
        Parameters: [
          BB.Parameter,
          ~r/^BB\.Parameter\./
        ],
        Kinematics: [
          BB.Motion,
          ~r/^BB\.Motion\./,
          ~r/^BB\.Robot\.Kinematics/,
          ~r/^BB\.IK\./
        ],
        Math: [
          ~r/^BB\.Math\./
        ],
        Errors: [
          BB.Error,
          ~r/^BB\.Error\./
        ],
        Collision: [
          BB.Collision,
          ~r/^BB\.Collision\./
        ],
        URDF: [
          ~r/^BB\.Urdf\./
        ],
        Simulation: [
          ~r/^BB\.Sim\./
        ],
        Bridges: [
          BB.Bridge,
          ~r/^BB\.Bridge/
        ],
        CLDR: [
          ~r/^BB\.Cldr/
        ],
        Examples: [
          ~r/^BB\.ExampleRobots/
        ],
        Testing: [
          ~r/^BB\.Test\./
        ],
        "Mix Tasks": [
          ~r/^Mix\.Tasks\.Bb/
        ],
        Units: [
          BB.Unit,
          ~r/^BB\.Unit\./
        ],
        Internals: [
          ~r/.*/
        ]
      ],
      source_ref: "main",
      source_url: "https://github.com/beam-bots/bb",
      before_closing_head_tag: &before_closing_head_tag/1
    ]
  end

  defp before_closing_head_tag(:html) do
    """
    <script src="https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.min.js"></script>
    <script>
      document.addEventListener("DOMContentLoaded", function () {
        mermaid.initialize({
          startOnLoad: false,
          theme: document.body.className.includes("dark") ? "dark" : "default"
        });
        let id = 0;
        for (const codeEl of document.querySelectorAll("pre code.mermaid")) {
          const preEl = codeEl.parentElement;
          const graphDefinition = codeEl.textContent;
          const graphEl = document.createElement("div");
          const graphId = "mermaid-graph-" + id++;
          mermaid.render(graphId, graphDefinition).then(({svg, bindFunctions}) => {
            graphEl.innerHTML = svg;
            bindFunctions?.(graphEl);
            preEl.insertAdjacentElement("afterend", graphEl);
            preEl.remove();
          });
        }
      });
    </script>
    """
  end

  defp before_closing_head_tag(:epub), do: ""

  defp aliases do
    [
      "spark.formatter": "spark.formatter --extensions BB.Dsl",
      "spark.cheat_sheets": "spark.cheat_sheets --extensions BB.Dsl"
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:ease, "~> 1.0"},
      {:ex_cldr_numbers, "~> 2.36"},
      {:ex_cldr_units, "~> 3.0"},
      {:nx, "~> 0.10"},
      {:spark, "~> 2.3"},
      {:splode, "~> 0.2"},

      # dev/test
      {:credo, "~> 1.7", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.4", only: [:dev, :test], runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:ex_doc, ">= 0.0.0", only: [:dev, :test], runtime: false},
      {:git_ops, "~> 2.9", only: [:dev, :test], runtime: false},
      {:igniter, "~> 0.6", optional: true},
      {:mimic, "~> 2.2", only: :test},
      {:mix_audit, "~> 2.1", only: [:dev, :test], runtime: false}
    ]
  end

  defp elixirc_paths(env) when env in [:dev, :test], do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]
end
