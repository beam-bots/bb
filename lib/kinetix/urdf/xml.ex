# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Urdf.Xml do
  @moduledoc """
  XML building utilities using Erlang's xmerl library.
  """

  @doc """
  Convert an xmerl element tree to an XML string with declaration.
  """
  @spec to_string(tuple()) :: String.t()
  def to_string(xml_tree) do
    [xml_tree]
    |> :xmerl.export_simple(:xmerl_xml)
    |> IO.iodata_to_binary()
  end

  @doc """
  Build an xmerl element tuple.

  Attributes are converted to charlists as required by xmerl.
  Nil children are filtered out.
  """
  @spec element(atom(), keyword(), list()) :: tuple()
  def element(name, attrs \\ [], children \\ []) do
    xmerl_attrs =
      attrs
      |> Enum.reject(fn {_k, v} -> is_nil(v) end)
      |> Enum.map(fn {k, v} -> {k, to_charlist(v)} end)

    xmerl_children =
      children
      |> List.flatten()
      |> Enum.reject(&is_nil/1)

    {name, xmerl_attrs, xmerl_children}
  end

  @doc """
  Format a 3-tuple as space-separated values.
  """
  @spec format_xyz({number(), number(), number()}) :: String.t()
  def format_xyz({x, y, z}) do
    "#{format_float(x)} #{format_float(y)} #{format_float(z)}"
  end

  @doc """
  Format a float with 6 decimal places, trimming trailing zeros.
  """
  @spec format_float(number()) :: String.t()
  def format_float(value) when is_number(value) do
    value
    |> :erlang.float_to_binary(decimals: 6)
    |> trim_trailing_zeros()
  end

  defp trim_trailing_zeros(str) do
    str
    |> String.trim_trailing("0")
    |> String.trim_trailing(".")
  end
end
