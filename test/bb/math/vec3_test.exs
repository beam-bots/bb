# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Vec3Test do
  use ExUnit.Case, async: true
  alias BB.Math.Vec3

  @tolerance 1.0e-6

  describe "new/3" do
    test "creates vector from components" do
      v = Vec3.new(1, 2, 3)

      assert_in_delta Vec3.x(v), 1.0, @tolerance
      assert_in_delta Vec3.y(v), 2.0, @tolerance
      assert_in_delta Vec3.z(v), 3.0, @tolerance
    end
  end

  describe "from_tensor/1" do
    test "wraps existing tensor" do
      t = Nx.tensor([4.0, 5.0, 6.0])
      v = Vec3.from_tensor(t)

      assert Vec3.to_list(v) == [4.0, 5.0, 6.0]
    end
  end

  describe "zero/0" do
    test "returns zero vector" do
      v = Vec3.zero()

      assert Vec3.to_list(v) == [0.0, 0.0, 0.0]
    end
  end

  describe "unit vectors" do
    test "unit_x returns (1, 0, 0)" do
      assert Vec3.to_list(Vec3.unit_x()) == [1.0, 0.0, 0.0]
    end

    test "unit_y returns (0, 1, 0)" do
      assert Vec3.to_list(Vec3.unit_y()) == [0.0, 1.0, 0.0]
    end

    test "unit_z returns (0, 0, 1)" do
      assert Vec3.to_list(Vec3.unit_z()) == [0.0, 0.0, 1.0]
    end
  end

  describe "add/2" do
    test "adds two vectors" do
      a = Vec3.new(1, 2, 3)
      b = Vec3.new(4, 5, 6)
      c = Vec3.add(a, b)

      assert Vec3.to_list(c) == [5.0, 7.0, 9.0]
    end
  end

  describe "subtract/2" do
    test "subtracts two vectors" do
      a = Vec3.new(4, 5, 6)
      b = Vec3.new(1, 2, 3)
      c = Vec3.subtract(a, b)

      assert Vec3.to_list(c) == [3.0, 3.0, 3.0]
    end
  end

  describe "negate/1" do
    test "negates a vector" do
      v = Vec3.new(1, -2, 3)
      n = Vec3.negate(v)

      assert Vec3.to_list(n) == [-1.0, 2.0, -3.0]
    end
  end

  describe "scale/2" do
    test "scales vector by scalar" do
      v = Vec3.new(1, 2, 3)
      s = Vec3.scale(v, 2)

      assert Vec3.to_list(s) == [2.0, 4.0, 6.0]
    end

    test "scales by negative" do
      v = Vec3.new(1, 2, 3)
      s = Vec3.scale(v, -1)

      assert Vec3.to_list(s) == [-1.0, -2.0, -3.0]
    end
  end

  describe "dot/2" do
    test "computes dot product" do
      a = Vec3.new(1, 2, 3)
      b = Vec3.new(4, 5, 6)

      assert_in_delta Vec3.dot(a, b), 32.0, @tolerance
    end

    test "perpendicular vectors have zero dot product" do
      a = Vec3.new(1, 0, 0)
      b = Vec3.new(0, 1, 0)

      assert_in_delta Vec3.dot(a, b), 0.0, @tolerance
    end
  end

  describe "cross/2" do
    test "X cross Y = Z" do
      x = Vec3.unit_x()
      y = Vec3.unit_y()
      z = Vec3.cross(x, y)

      assert_in_delta Vec3.x(z), 0.0, @tolerance
      assert_in_delta Vec3.y(z), 0.0, @tolerance
      assert_in_delta Vec3.z(z), 1.0, @tolerance
    end

    test "Y cross Z = X" do
      y = Vec3.unit_y()
      z = Vec3.unit_z()
      x = Vec3.cross(y, z)

      assert_in_delta Vec3.x(x), 1.0, @tolerance
      assert_in_delta Vec3.y(x), 0.0, @tolerance
      assert_in_delta Vec3.z(x), 0.0, @tolerance
    end

    test "Z cross X = Y" do
      z = Vec3.unit_z()
      x = Vec3.unit_x()
      y = Vec3.cross(z, x)

      assert_in_delta Vec3.x(y), 0.0, @tolerance
      assert_in_delta Vec3.y(y), 1.0, @tolerance
      assert_in_delta Vec3.z(y), 0.0, @tolerance
    end

    test "cross product is anti-commutative" do
      a = Vec3.new(1, 2, 3)
      b = Vec3.new(4, 5, 6)

      ab = Vec3.cross(a, b)
      ba = Vec3.cross(b, a)

      assert_in_delta Vec3.x(ab), -Vec3.x(ba), @tolerance
      assert_in_delta Vec3.y(ab), -Vec3.y(ba), @tolerance
      assert_in_delta Vec3.z(ab), -Vec3.z(ba), @tolerance
    end
  end

  describe "magnitude/1" do
    test "computes vector length" do
      v = Vec3.new(3, 4, 0)

      assert_in_delta Vec3.magnitude(v), 5.0, @tolerance
    end

    test "unit vector has magnitude 1" do
      v = Vec3.unit_x()

      assert_in_delta Vec3.magnitude(v), 1.0, @tolerance
    end

    test "zero vector has magnitude 0" do
      v = Vec3.zero()

      assert_in_delta Vec3.magnitude(v), 0.0, @tolerance
    end
  end

  describe "magnitude_squared/1" do
    test "computes squared length" do
      v = Vec3.new(3, 4, 0)

      assert_in_delta Vec3.magnitude_squared(v), 25.0, @tolerance
    end
  end

  describe "normalise/1" do
    test "normalises vector to unit length" do
      v = Vec3.new(3, 0, 0)
      n = Vec3.normalise(v)

      assert_in_delta Vec3.magnitude(n), 1.0, @tolerance
      assert_in_delta Vec3.x(n), 1.0, @tolerance
    end

    test "normalises arbitrary vector" do
      v = Vec3.new(1, 2, 3)
      n = Vec3.normalise(v)

      assert_in_delta Vec3.magnitude(n), 1.0, @tolerance
    end

    test "zero vector normalises to zero" do
      v = Vec3.zero()
      n = Vec3.normalise(v)

      assert_in_delta Vec3.magnitude(n), 0.0, @tolerance
    end
  end

  describe "distance/2" do
    test "computes distance between points" do
      a = Vec3.new(0, 0, 0)
      b = Vec3.new(3, 4, 0)

      assert_in_delta Vec3.distance(a, b), 5.0, @tolerance
    end

    test "distance is symmetric" do
      a = Vec3.new(1, 2, 3)
      b = Vec3.new(4, 5, 6)

      assert_in_delta Vec3.distance(a, b), Vec3.distance(b, a), @tolerance
    end
  end

  describe "lerp/3" do
    test "t=0 returns first vector" do
      a = Vec3.new(0, 0, 0)
      b = Vec3.new(10, 10, 10)
      c = Vec3.lerp(a, b, 0)

      assert Vec3.to_list(c) == [0.0, 0.0, 0.0]
    end

    test "t=1 returns second vector" do
      a = Vec3.new(0, 0, 0)
      b = Vec3.new(10, 10, 10)
      c = Vec3.lerp(a, b, 1)

      assert Vec3.to_list(c) == [10.0, 10.0, 10.0]
    end

    test "t=0.5 returns midpoint" do
      a = Vec3.new(0, 0, 0)
      b = Vec3.new(10, 10, 10)
      c = Vec3.lerp(a, b, 0.5)

      assert Vec3.to_list(c) == [5.0, 5.0, 5.0]
    end
  end

  describe "tensor/1" do
    test "extracts underlying tensor" do
      v = Vec3.new(1, 2, 3)
      t = Vec3.tensor(v)

      assert Nx.shape(t) == {3}
      assert Nx.to_flat_list(t) == [1.0, 2.0, 3.0]
    end
  end
end
