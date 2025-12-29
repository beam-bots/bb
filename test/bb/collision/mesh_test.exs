# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Collision.MeshTest do
  use ExUnit.Case, async: true

  alias BB.Collision.Mesh
  alias BB.Math.Vec3

  @fixtures_dir Path.join([__DIR__, "..", "..", "fixtures"])

  setup do
    Mesh.clear_cache()
    :ok
  end

  describe "load_bounds/1" do
    test "loads ASCII STL file" do
      path = Path.join(@fixtures_dir, "cube.stl")
      assert {:ok, bounds} = Mesh.load_bounds(path)

      {min_pt, max_pt} = bounds.aabb
      assert_in_delta Vec3.x(min_pt), 0.0, 0.001
      assert_in_delta Vec3.y(min_pt), 0.0, 0.001
      assert_in_delta Vec3.z(min_pt), 0.0, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 1.0, 0.001
      assert_in_delta Vec3.z(max_pt), 1.0, 0.001
    end

    test "computes bounding sphere" do
      path = Path.join(@fixtures_dir, "cube.stl")
      assert {:ok, bounds} = Mesh.load_bounds(path)

      {centre, radius} = bounds.bounding_sphere

      # Centre should be at (0.5, 0.5, 0.5)
      assert_in_delta Vec3.x(centre), 0.5, 0.001
      assert_in_delta Vec3.y(centre), 0.5, 0.001
      assert_in_delta Vec3.z(centre), 0.5, 0.001

      # Radius should be distance from centre to corner: sqrt(0.5^2 * 3) ≈ 0.866
      expected_radius = :math.sqrt(0.75)
      assert_in_delta radius, expected_radius, 0.001
    end

    test "caches results" do
      path = Path.join(@fixtures_dir, "cube.stl")

      # First load
      {:ok, bounds1} = Mesh.load_bounds(path)

      # Second load should return same result (from cache)
      {:ok, bounds2} = Mesh.load_bounds(path)

      assert bounds1 == bounds2
    end

    test "returns error for non-existent file" do
      assert {:error, {:file_stat_error, :enoent}} = Mesh.load_bounds("/nonexistent/file.stl")
    end
  end

  describe "load_bounds!/1" do
    test "returns bounds for valid file" do
      path = Path.join(@fixtures_dir, "cube.stl")
      bounds = Mesh.load_bounds!(path)

      assert is_map(bounds)
      assert Map.has_key?(bounds, :aabb)
      assert Map.has_key?(bounds, :bounding_sphere)
    end

    test "raises for invalid file" do
      assert_raise RuntimeError, ~r/Failed to load mesh bounds/, fn ->
        Mesh.load_bounds!("/nonexistent/file.stl")
      end
    end
  end

  describe "compute_bounds/1" do
    test "computes AABB from vertices" do
      vertices = [
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 0.0, 4.0}
      ]

      {:ok, bounds} = Mesh.compute_bounds(vertices)
      {min_pt, max_pt} = bounds.aabb

      assert_in_delta Vec3.x(min_pt), 0.0, 0.001
      assert_in_delta Vec3.y(min_pt), 0.0, 0.001
      assert_in_delta Vec3.z(min_pt), 0.0, 0.001
      assert_in_delta Vec3.x(max_pt), 2.0, 0.001
      assert_in_delta Vec3.y(max_pt), 3.0, 0.001
      assert_in_delta Vec3.z(max_pt), 4.0, 0.001
    end

    test "computes bounding sphere from vertices" do
      vertices = [
        {-1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 1.0, 0.0}
      ]

      {:ok, bounds} = Mesh.compute_bounds(vertices)
      {centre, radius} = bounds.bounding_sphere

      # Centre should be at origin
      assert_in_delta Vec3.x(centre), 0.0, 0.001
      assert_in_delta Vec3.y(centre), 0.0, 0.001
      assert_in_delta Vec3.z(centre), 0.0, 0.001

      # Radius should be 1.0
      assert_in_delta radius, 1.0, 0.001
    end

    test "handles single vertex" do
      vertices = [{5.0, 5.0, 5.0}]

      {:ok, bounds} = Mesh.compute_bounds(vertices)
      {min_pt, max_pt} = bounds.aabb
      {centre, radius} = bounds.bounding_sphere

      assert_in_delta Vec3.x(min_pt), 5.0, 0.001
      assert_in_delta Vec3.x(max_pt), 5.0, 0.001
      assert_in_delta Vec3.x(centre), 5.0, 0.001
      assert_in_delta radius, 0.0, 0.001
    end

    test "returns error for empty vertex list" do
      assert {:error, :empty_mesh} = Mesh.compute_bounds([])
    end

    test "handles negative coordinates" do
      vertices = [
        {-2.0, -3.0, -4.0},
        {1.0, 2.0, 3.0}
      ]

      {:ok, bounds} = Mesh.compute_bounds(vertices)
      {min_pt, max_pt} = bounds.aabb

      assert_in_delta Vec3.x(min_pt), -2.0, 0.001
      assert_in_delta Vec3.y(min_pt), -3.0, 0.001
      assert_in_delta Vec3.z(min_pt), -4.0, 0.001
      assert_in_delta Vec3.x(max_pt), 1.0, 0.001
      assert_in_delta Vec3.y(max_pt), 2.0, 0.001
      assert_in_delta Vec3.z(max_pt), 3.0, 0.001
    end
  end

  describe "clear_cache/0" do
    test "clears cached bounds" do
      path = Path.join(@fixtures_dir, "cube.stl")

      # Load to populate cache
      {:ok, _} = Mesh.load_bounds(path)

      # Clear cache
      :ok = Mesh.clear_cache()

      # Should still work (reloads from file)
      {:ok, _} = Mesh.load_bounds(path)
    end
  end

  describe "binary STL parsing" do
    test "parses binary STL" do
      # Create a minimal binary STL in memory with one triangle
      header = String.duplicate(<<0>>, 80)
      num_triangles = <<1::little-unsigned-32>>

      # Normal (0, 0, 1)
      normal = <<0.0::little-float-32, 0.0::little-float-32, 1.0::little-float-32>>

      # Vertices forming a triangle
      v1 = <<0.0::little-float-32, 0.0::little-float-32, 0.0::little-float-32>>
      v2 = <<1.0::little-float-32, 0.0::little-float-32, 0.0::little-float-32>>
      v3 = <<0.0::little-float-32, 1.0::little-float-32, 0.0::little-float-32>>

      # Attribute byte count
      attr = <<0::16>>

      binary_stl = header <> num_triangles <> normal <> v1 <> v2 <> v3 <> attr

      # Write to temp file
      path = Path.join(System.tmp_dir!(), "test_binary_#{:erlang.unique_integer([:positive])}.stl")
      File.write!(path, binary_stl)

      try do
        {:ok, bounds} = Mesh.load_bounds(path)
        {min_pt, max_pt} = bounds.aabb

        assert_in_delta Vec3.x(min_pt), 0.0, 0.001
        assert_in_delta Vec3.y(min_pt), 0.0, 0.001
        assert_in_delta Vec3.z(min_pt), 0.0, 0.001
        assert_in_delta Vec3.x(max_pt), 1.0, 0.001
        assert_in_delta Vec3.y(max_pt), 1.0, 0.001
        assert_in_delta Vec3.z(max_pt), 0.0, 0.001
      after
        File.rm(path)
      end
    end
  end
end
