# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision.Mesh do
  @moduledoc """
  Mesh loading and bounding geometry computation for collision detection.

  This module provides basic mesh support for collision detection by computing
  bounding primitives (spheres or AABBs) from mesh vertices. Triangle-level
  collision detection is not supported - meshes are approximated by their
  bounding geometry.

  Currently supports:
  - Binary STL files
  - ASCII STL files

  ## Usage

      # Load mesh and compute bounds
      {:ok, bounds} = BB.Collision.Mesh.load_bounds("/path/to/model.stl")

      # bounds contains:
      %{
        aabb: {min_vec3, max_vec3},
        bounding_sphere: {centre_vec3, radius}
      }

  ## Caching

  Mesh bounds are cached in an ETS table to avoid repeated file parsing.
  The cache key includes the file path and modification time.
  """

  alias BB.Math.Vec3

  @type bounds :: %{
          aabb: {Vec3.t(), Vec3.t()},
          bounding_sphere: {Vec3.t(), float()}
        }

  @cache_table :bb_collision_mesh_cache

  @doc """
  Load mesh bounds from a file.

  Parses the mesh file, computes bounding geometry, and caches the result.
  Subsequent calls with the same file (unchanged) return cached bounds.

  Returns `{:ok, bounds}` or `{:error, reason}`.
  """
  @spec load_bounds(String.t()) :: {:ok, bounds()} | {:error, term()}
  def load_bounds(path) do
    with {:ok, cache_key} <- get_cache_key(path) do
      case get_cached(cache_key) do
        {:ok, bounds} ->
          {:ok, bounds}

        :miss ->
          with {:ok, vertices} <- parse_mesh(path),
               {:ok, bounds} <- compute_bounds(vertices) do
            put_cached(cache_key, bounds)
            {:ok, bounds}
          end
      end
    end
  end

  @doc """
  Load mesh bounds, raising on error.
  """
  @spec load_bounds!(String.t()) :: bounds()
  def load_bounds!(path) do
    case load_bounds(path) do
      {:ok, bounds} -> bounds
      {:error, reason} -> raise "Failed to load mesh bounds: #{inspect(reason)}"
    end
  end

  @doc """
  Clear the mesh bounds cache.
  """
  @spec clear_cache() :: :ok
  def clear_cache do
    ensure_cache_table()
    :ets.delete_all_objects(@cache_table)
    :ok
  end

  @doc """
  Compute bounding geometry from a list of vertices.

  Each vertex should be a `{x, y, z}` tuple of floats.
  """
  @spec compute_bounds([{float(), float(), float()}]) :: {:ok, bounds()} | {:error, :empty_mesh}
  def compute_bounds([]), do: {:error, :empty_mesh}

  def compute_bounds(vertices) when is_list(vertices) do
    aabb = compute_aabb(vertices)
    bounding_sphere = compute_bounding_sphere(vertices, aabb)
    {:ok, %{aabb: aabb, bounding_sphere: bounding_sphere}}
  end

  # ============================================================================
  # STL Parsing
  # ============================================================================

  defp parse_mesh(path) do
    case File.read(path) do
      {:ok, data} ->
        parse_stl(data)

      {:error, reason} ->
        {:error, {:file_read_error, reason}}
    end
  end

  defp parse_stl(<<"solid ", rest::binary>>) do
    # Could be ASCII STL, but binary STL can also start with "solid"
    # Check if it looks like ASCII by finding "facet" keyword
    if String.contains?(rest, "facet") do
      parse_ascii_stl(<<"solid ", rest::binary>>)
    else
      parse_binary_stl(<<"solid ", rest::binary>>)
    end
  end

  defp parse_stl(data), do: parse_binary_stl(data)

  defp parse_binary_stl(data) when byte_size(data) < 84 do
    {:error, :invalid_stl}
  end

  defp parse_binary_stl(
         <<_header::binary-size(80), num_triangles::little-unsigned-32, rest::binary>>
       ) do
    expected_size = num_triangles * 50

    if byte_size(rest) >= expected_size do
      vertices = parse_binary_triangles(rest, num_triangles, [])
      {:ok, vertices}
    else
      {:error, :truncated_stl}
    end
  end

  defp parse_binary_triangles(_data, 0, acc), do: Enum.uniq(acc)

  defp parse_binary_triangles(
         <<_nx::little-float-32, _ny::little-float-32, _nz::little-float-32, v1x::little-float-32,
           v1y::little-float-32, v1z::little-float-32, v2x::little-float-32, v2y::little-float-32,
           v2z::little-float-32, v3x::little-float-32, v3y::little-float-32, v3z::little-float-32,
           _attr::binary-size(2), rest::binary>>,
         remaining,
         acc
       ) do
    vertices = [
      {v1x, v1y, v1z},
      {v2x, v2y, v2z},
      {v3x, v3y, v3z}
      | acc
    ]

    parse_binary_triangles(rest, remaining - 1, vertices)
  end

  defp parse_ascii_stl(data) do
    # Extract all vertex lines
    vertices =
      data
      |> String.split("\n")
      |> Enum.filter(&String.contains?(&1, "vertex"))
      |> Enum.map(&parse_vertex_line/1)
      |> Enum.reject(&is_nil/1)
      |> Enum.uniq()

    if Enum.empty?(vertices) do
      {:error, :no_vertices_found}
    else
      {:ok, vertices}
    end
  end

  defp parse_vertex_line(line) do
    case Regex.run(~r/vertex\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)/, line) do
      [_, x, y, z] ->
        {parse_float(x), parse_float(y), parse_float(z)}

      _ ->
        nil
    end
  end

  defp parse_float(str) do
    case Float.parse(str) do
      {f, _} -> f
      :error -> 0.0
    end
  end

  # ============================================================================
  # Bounding Geometry Computation
  # ============================================================================

  defp compute_aabb(vertices) do
    {xs, ys, zs} =
      Enum.reduce(vertices, {[], [], []}, fn {x, y, z}, {xs, ys, zs} ->
        {[x | xs], [y | ys], [z | zs]}
      end)

    min_pt = Vec3.new(Enum.min(xs), Enum.min(ys), Enum.min(zs))
    max_pt = Vec3.new(Enum.max(xs), Enum.max(ys), Enum.max(zs))

    {min_pt, max_pt}
  end

  defp compute_bounding_sphere(vertices, {min_pt, max_pt}) do
    # Use Ritter's algorithm for a reasonable bounding sphere
    # Start with AABB centre, then expand to contain all points

    centre = Vec3.lerp(min_pt, max_pt, 0.5)

    # Find the maximum distance from centre to any vertex
    radius =
      vertices
      |> Enum.map(fn {x, y, z} ->
        vertex = Vec3.new(x, y, z)
        Vec3.distance(centre, vertex)
      end)
      |> Enum.max()

    {centre, radius}
  end

  # ============================================================================
  # Caching
  # ============================================================================

  defp get_cache_key(path) do
    case File.stat(path) do
      {:ok, %{mtime: mtime}} ->
        {:ok, {path, mtime}}

      {:error, reason} ->
        {:error, {:file_stat_error, reason}}
    end
  end

  defp ensure_cache_table do
    if :ets.whereis(@cache_table) == :undefined do
      :ets.new(@cache_table, [:named_table, :set, :public, read_concurrency: true])
    end

    :ok
  end

  defp get_cached(key) do
    ensure_cache_table()

    case :ets.lookup(@cache_table, key) do
      [{^key, bounds}] -> {:ok, bounds}
      [] -> :miss
    end
  end

  defp put_cached(key, bounds) do
    ensure_cache_table()
    :ets.insert(@cache_table, {key, bounds})
    :ok
  end
end
