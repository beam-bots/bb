# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.Bb.ToUrdf do
  @shortdoc "Export a BB robot definition to URDF format"

  @moduledoc """
  Export a BB robot definition to URDF XML format.

  ## Usage

      mix bb.to_urdf MyApp.Robot --output robot.urdf
      mix bb.to_urdf MyApp.Robot -o -

  ## Options

    * `--output`, `-o` - Output file path. Use `-` for stdout.
      If not specified, prints to stdout.
  """

  use Mix.Task

  alias BB.Urdf.Exporter

  @switches [output: :string]
  @aliases [o: :output]

  @impl Mix.Task
  def run(args) do
    Mix.Task.run("compile")

    case parse_args(args) do
      {:ok, module, opts} ->
        export_robot(module, opts)

      {:error, message} ->
        Mix.shell().error(message)
        exit({:shutdown, 1})
    end
  end

  defp parse_args(args) do
    {opts, rest, invalid} = OptionParser.parse(args, strict: @switches, aliases: @aliases)

    cond do
      invalid != [] ->
        {opt, _} = hd(invalid)
        {:error, "Unknown option: #{opt}"}

      rest == [] ->
        {:error, "Usage: mix bb.to_urdf MODULE [--output FILE]"}

      length(rest) > 1 ->
        {:error, "Too many arguments. Expected one module name."}

      true ->
        {:ok, parse_module(hd(rest)), opts}
    end
  end

  defp parse_module(module_string) do
    module_string
    |> String.split(".")
    |> Enum.map(&String.to_atom/1)
    |> Module.concat()
  end

  defp export_robot(module, opts) do
    case Exporter.export(module) do
      {:ok, xml} ->
        write_output(xml, opts[:output])

      {:error, {:module_not_found, mod, reason}} ->
        Mix.shell().error("Module #{inspect(mod)} could not be loaded: #{inspect(reason)}")
        exit({:shutdown, 1})

      {:error, {:not_a_bb_module, mod}} ->
        Mix.shell().error("Module #{inspect(mod)} does not use BB (no robot/0 function)")
        exit({:shutdown, 1})
    end
  end

  defp write_output(xml, nil), do: Mix.shell().info(xml)
  defp write_output(xml, "-"), do: Mix.shell().info(xml)

  defp write_output(xml, path) do
    case File.write(path, xml) do
      :ok ->
        Mix.shell().info("Wrote URDF to #{path}")

      {:error, reason} ->
        Mix.shell().error("Failed to write #{path}: #{inspect(reason)}")
        exit({:shutdown, 1})
    end
  end
end
