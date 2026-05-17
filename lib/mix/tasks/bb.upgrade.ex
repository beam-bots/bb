# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.Bb.Upgrade do
    @shortdoc "Run upgrade tasks for `bb` between two versions"
    @moduledoc """
    #{@shortdoc}

    Usually invoked indirectly via `mix igniter.upgrade bb`.

    ## Example

    ```bash
    mix igniter.upgrade bb
    ```
    """

    use Igniter.Mix.Task

    alias BB.Igniter.Upgrade.V0_16

    @impl Igniter.Mix.Task
    def info(_argv, _source) do
      %Igniter.Mix.Task.Info{
        group: :bb,
        positional: [:from, :to]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      positional = igniter.args.positional

      upgrades = %{
        "0.16.0" => [
          &V0_16.remove_auto_disarm_on_error/2,
          &V0_16.rename_bb_cldr_unit_alias/2,
          &V0_16.rewrite_cldr_unit_calls/2,
          &V0_16.rewrite_cldr_unit_struct_patterns/2,
          &V0_16.add_release_notice/2
        ]
      }

      Igniter.Upgrades.run(igniter, positional.from, positional.to, upgrades,
        custom_opts: igniter.args.options
      )
    end
  end
else
  defmodule Mix.Tasks.Bb.Upgrade do
    @moduledoc false

    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The task `bb.upgrade` requires `:igniter`. Add it to your dependencies and try again.

      For more information, see: https://hexdocs.pm/igniter/readme.html#installation
      """)

      exit({:shutdown, 1})
    end
  end
end
