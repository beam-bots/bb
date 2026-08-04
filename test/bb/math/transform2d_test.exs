# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Transform2DTest do
  use ExUnit.Case, async: true
  doctest BB.Math.Transform2D

  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3

  @normals [
    Vec3.unit_x(),
    Vec3.unit_y(),
    Vec3.unit_z(),
    Vec3.normalise(Vec3.new(1.0, 1.0, 1.0)),
    Vec3.normalise(Vec3.new(1.0, 0.0, 2.0))
  ]

  defp matrix(%Transform{} = t), do: t |> Transform.tensor() |> Nx.to_flat_list()

  defp assert_transforms_equal(a, b) do
    Enum.zip(matrix(a), matrix(b))
    |> Enum.each(fn {left, right} -> assert_in_delta left, right, 1.0e-12 end)
  end

  describe "new/3" do
    test "coerces integers to floats" do
      assert %Transform2D{x: 1.0, y: 2.0, theta: +0.0} = Transform2D.new(1, 2, 0)
    end
  end

  describe "compose/2" do
    test "identity is a left and right unit" do
      t = Transform2D.new(1.0, 2.0, 0.5)

      assert Transform2D.compose(Transform2D.identity(), t) == t
      assert Transform2D.compose(t, Transform2D.identity()) == t
    end

    test "rotates the second transform's translation by the first's angle" do
      a = Transform2D.new(1.0, 0.0, :math.pi() / 2)
      b = Transform2D.new(1.0, 0.0, 0.0)

      c = Transform2D.compose(a, b)

      assert_in_delta c.x, 1.0, 1.0e-12
      assert_in_delta c.y, 1.0, 1.0e-12
    end
  end

  describe "inverse/1" do
    test "composes to the identity" do
      t = Transform2D.new(1.5, -2.5, 0.9)
      c = Transform2D.compose(t, Transform2D.inverse(t))

      assert_in_delta c.x, 0.0, 1.0e-12
      assert_in_delta c.y, 0.0, 1.0e-12
      assert_in_delta c.theta, 0.0, 1.0e-12
    end
  end

  describe "to_transform/2" do
    test "a Z normal reduces to the XY plane with theta as yaw" do
      t = Transform2D.to_transform(Transform2D.new(1.0, 2.0, :math.pi() / 2), Vec3.unit_z())

      translation = t |> Transform.get_translation() |> Vec3.to_list()
      assert Enum.map(translation, &Float.round(&1, 9)) == [1.0, 2.0, 0.0]

      rotated = Transform.apply_to_point(t, Vec3.unit_x())
      assert Enum.map(Vec3.to_list(rotated), &Float.round(&1, 9)) == [1.0, 3.0, 0.0]
    end

    test "the plane basis is orthonormal and right-handed about the normal" do
      for normal <- @normals do
        u = in_plane_axis(Transform2D.new(1.0, 0.0, 0.0), normal)
        v = in_plane_axis(Transform2D.new(0.0, 1.0, 0.0), normal)

        assert_in_delta Vec3.magnitude(u), 1.0, 1.0e-12
        assert_in_delta Vec3.magnitude(v), 1.0, 1.0e-12
        assert_in_delta Vec3.dot(u, normal), 0.0, 1.0e-12
        assert_in_delta Vec3.dot(v, normal), 0.0, 1.0e-12
        assert_in_delta Vec3.dot(u, v), 0.0, 1.0e-12

        Enum.zip(Vec3.to_list(Vec3.cross(u, v)), Vec3.to_list(normal))
        |> Enum.each(fn {left, right} -> assert_in_delta left, right, 1.0e-12 end)
      end
    end

    test "theta rotates about the normal, leaving it fixed" do
      for normal <- @normals do
        t = Transform2D.to_transform(Transform2D.new(0.0, 0.0, 0.7), normal)

        Enum.zip(Vec3.to_list(Transform.apply_to_point(t, normal)), Vec3.to_list(normal))
        |> Enum.each(fn {left, right} -> assert_in_delta left, right, 1.0e-12 end)
      end
    end

    # Lifting the composition must equal composing the liftings, or a planar
    # joint's kinematics would disagree with its own configuration algebra.
    test "agrees with Transform.compose/2" do
      a = Transform2D.new(1.0, 2.0, 0.7)
      b = Transform2D.new(-0.5, 0.25, -0.3)

      for normal <- @normals do
        assert_transforms_equal(
          Transform2D.to_transform(Transform2D.compose(a, b), normal),
          Transform.compose(
            Transform2D.to_transform(a, normal),
            Transform2D.to_transform(b, normal)
          )
        )
      end
    end

    test "agrees with Transform.inverse/1" do
      t = Transform2D.new(1.5, -2.5, 0.9)

      for normal <- @normals do
        assert_transforms_equal(
          Transform2D.to_transform(Transform2D.inverse(t), normal),
          Transform.inverse(Transform2D.to_transform(t, normal))
        )
      end
    end

    test "the identity lifts to the identity" do
      for normal <- @normals do
        assert_transforms_equal(
          Transform2D.to_transform(Transform2D.identity(), normal),
          Transform.identity()
        )
      end
    end

    test "normalises the given normal" do
      unit = Transform2D.to_transform(Transform2D.new(1.0, 2.0, 0.4), Vec3.unit_z())
      scaled = Transform2D.to_transform(Transform2D.new(1.0, 2.0, 0.4), Vec3.new(0.0, 0.0, 5.0))

      assert_transforms_equal(unit, scaled)
    end
  end

  defp in_plane_axis(%Transform2D{} = t, normal) do
    t |> Transform2D.to_transform(normal) |> Transform.get_translation()
  end
end
