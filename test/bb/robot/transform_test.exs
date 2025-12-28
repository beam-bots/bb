# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.TransformTest do
  use ExUnit.Case, async: true
  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Robot.Transform

  @tolerance 1.0e-6

  describe "from_quaternion/1" do
    test "identity quaternion produces identity rotation" do
      q = Quaternion.identity()
      t = Transform.from_quaternion(q)

      assert_in_delta Nx.to_number(t[0][0]), 1.0, @tolerance
      assert_in_delta Nx.to_number(t[1][1]), 1.0, @tolerance
      assert_in_delta Nx.to_number(t[2][2]), 1.0, @tolerance
      assert_in_delta Nx.to_number(t[3][3]), 1.0, @tolerance

      assert_in_delta Nx.to_number(t[0][3]), 0.0, @tolerance
      assert_in_delta Nx.to_number(t[1][3]), 0.0, @tolerance
      assert_in_delta Nx.to_number(t[2][3]), 0.0, @tolerance
    end

    test "90 degree rotation around Z axis" do
      q = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 2)
      t = Transform.from_quaternion(q)

      {x, y, z} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})

      assert_in_delta x, 0.0, @tolerance
      assert_in_delta y, 1.0, @tolerance
      assert_in_delta z, 0.0, @tolerance
    end

    test "90 degree rotation around X axis" do
      q = Quaternion.from_axis_angle(Vec3.unit_x(), :math.pi() / 2)
      t = Transform.from_quaternion(q)

      {x, y, z} = Transform.apply_to_point(t, {0.0, 1.0, 0.0})

      assert_in_delta x, 0.0, @tolerance
      assert_in_delta y, 0.0, @tolerance
      assert_in_delta z, 1.0, @tolerance
    end

    test "90 degree rotation around Y axis" do
      q = Quaternion.from_axis_angle(Vec3.unit_y(), :math.pi() / 2)
      t = Transform.from_quaternion(q)

      {x, y, z} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})

      assert_in_delta x, 0.0, @tolerance
      assert_in_delta y, 0.0, @tolerance
      assert_in_delta z, -1.0, @tolerance
    end

    test "translation component is zero" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 1, 1), 0.5)
      t = Transform.from_quaternion(q)
      {tx, ty, tz} = Transform.get_translation(t)

      assert_in_delta tx, 0.0, @tolerance
      assert_in_delta ty, 0.0, @tolerance
      assert_in_delta tz, 0.0, @tolerance
    end
  end

  describe "get_quaternion/1" do
    test "extracts quaternion from identity transform" do
      t = Transform.identity()
      q = Transform.get_quaternion(t)

      assert_in_delta Quaternion.w(q), 1.0, @tolerance
      assert_in_delta Quaternion.x(q), 0.0, @tolerance
      assert_in_delta Quaternion.y(q), 0.0, @tolerance
      assert_in_delta Quaternion.z(q), 0.0, @tolerance
    end

    test "extracts quaternion from rotation_z" do
      angle = :math.pi() / 2
      t = Transform.rotation_z(angle)
      q = Transform.get_quaternion(t)
      {axis, extracted_angle} = Quaternion.to_axis_angle(q)

      assert_in_delta extracted_angle, angle, @tolerance
      assert_in_delta Vec3.z(axis), 1.0, @tolerance
    end

    test "extracts quaternion from rotation_x" do
      angle = :math.pi() / 3
      t = Transform.rotation_x(angle)
      q = Transform.get_quaternion(t)
      {axis, extracted_angle} = Quaternion.to_axis_angle(q)

      assert_in_delta extracted_angle, angle, @tolerance
      assert_in_delta Vec3.x(axis), 1.0, @tolerance
    end

    test "extracts quaternion from rotation_y" do
      angle = :math.pi() / 4
      t = Transform.rotation_y(angle)
      q = Transform.get_quaternion(t)
      {axis, extracted_angle} = Quaternion.to_axis_angle(q)

      assert_in_delta extracted_angle, angle, @tolerance
      assert_in_delta Vec3.y(axis), 1.0, @tolerance
    end

    test "round-trip: from_quaternion -> get_quaternion" do
      original = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 0.7)
      t = Transform.from_quaternion(original)
      recovered = Transform.get_quaternion(t)

      dist = Quaternion.angular_distance(original, recovered)
      assert_in_delta dist, 0.0, 1.0e-3
    end
  end

  describe "from_position_quaternion/2" do
    test "with identity quaternion preserves position" do
      pos = Vec3.new(1.0, 2.0, 3.0)
      q = Quaternion.identity()
      t = Transform.from_position_quaternion(pos, q)

      {tx, ty, tz} = Transform.get_translation(t)
      assert_in_delta tx, 1.0, @tolerance
      assert_in_delta ty, 2.0, @tolerance
      assert_in_delta tz, 3.0, @tolerance
    end

    test "with rotation applies both position and orientation" do
      pos = Vec3.new(5.0, 0.0, 0.0)
      q = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 2)
      t = Transform.from_position_quaternion(pos, q)

      {x, y, z} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})
      assert_in_delta x, 5.0, @tolerance
      assert_in_delta y, 1.0, @tolerance
      assert_in_delta z, 0.0, @tolerance
    end

    test "position and orientation are independent" do
      pos = Vec3.new(10.0, 20.0, 30.0)
      q = Quaternion.from_axis_angle(Vec3.unit_x(), :math.pi() / 4)
      t = Transform.from_position_quaternion(pos, q)

      {tx, ty, tz} = Transform.get_translation(t)
      assert_in_delta tx, 10.0, @tolerance
      assert_in_delta ty, 20.0, @tolerance
      assert_in_delta tz, 30.0, @tolerance

      recovered_q = Transform.get_quaternion(t)
      dist = Quaternion.angular_distance(q, recovered_q)
      assert_in_delta dist, 0.0, 1.0e-3
    end
  end

  describe "get_forward_vector/1" do
    test "identity transform has Z forward" do
      t = Transform.identity()
      fwd = Transform.get_forward_vector(t)

      assert_in_delta Vec3.x(fwd), 0.0, @tolerance
      assert_in_delta Vec3.y(fwd), 0.0, @tolerance
      assert_in_delta Vec3.z(fwd), 1.0, @tolerance
    end

    test "rotation around Y rotates forward vector" do
      t = Transform.rotation_y(:math.pi() / 2)
      fwd = Transform.get_forward_vector(t)

      assert_in_delta Vec3.x(fwd), 1.0, @tolerance
      assert_in_delta Vec3.y(fwd), 0.0, @tolerance
      assert_in_delta Vec3.z(fwd), 0.0, @tolerance
    end

    test "rotation around X rotates forward vector" do
      t = Transform.rotation_x(:math.pi() / 2)
      fwd = Transform.get_forward_vector(t)

      assert_in_delta Vec3.x(fwd), 0.0, @tolerance
      assert_in_delta Vec3.y(fwd), -1.0, @tolerance
      assert_in_delta Vec3.z(fwd), 0.0, @tolerance
    end
  end

  describe "get_up_vector/1" do
    test "identity transform has Y up" do
      t = Transform.identity()
      up = Transform.get_up_vector(t)

      assert_in_delta Vec3.x(up), 0.0, @tolerance
      assert_in_delta Vec3.y(up), 1.0, @tolerance
      assert_in_delta Vec3.z(up), 0.0, @tolerance
    end

    test "rotation around X rotates up vector" do
      t = Transform.rotation_x(:math.pi() / 2)
      up = Transform.get_up_vector(t)

      assert_in_delta Vec3.x(up), 0.0, @tolerance
      assert_in_delta Vec3.y(up), 0.0, @tolerance
      assert_in_delta Vec3.z(up), 1.0, @tolerance
    end

    test "rotation around Z rotates up vector" do
      t = Transform.rotation_z(:math.pi() / 2)
      up = Transform.get_up_vector(t)

      assert_in_delta Vec3.x(up), -1.0, @tolerance
      assert_in_delta Vec3.y(up), 0.0, @tolerance
      assert_in_delta Vec3.z(up), 0.0, @tolerance
    end
  end

  describe "get_right_vector/1" do
    test "identity transform has X right" do
      t = Transform.identity()
      right = Transform.get_right_vector(t)

      assert_in_delta Vec3.x(right), 1.0, @tolerance
      assert_in_delta Vec3.y(right), 0.0, @tolerance
      assert_in_delta Vec3.z(right), 0.0, @tolerance
    end

    test "rotation around Z rotates right vector" do
      t = Transform.rotation_z(:math.pi() / 2)
      right = Transform.get_right_vector(t)

      assert_in_delta Vec3.x(right), 0.0, @tolerance
      assert_in_delta Vec3.y(right), 1.0, @tolerance
      assert_in_delta Vec3.z(right), 0.0, @tolerance
    end

    test "rotation around Y rotates right vector" do
      t = Transform.rotation_y(:math.pi() / 2)
      right = Transform.get_right_vector(t)

      # +90° around Y takes X toward -Z (right-hand rule)
      assert_in_delta Vec3.x(right), 0.0, @tolerance
      assert_in_delta Vec3.y(right), 0.0, @tolerance
      assert_in_delta Vec3.z(right), -1.0, @tolerance
    end
  end

  describe "axis vectors are orthonormal" do
    test "all axis vectors are unit length" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 0.8)
      t = Transform.from_quaternion(q)

      right = Transform.get_right_vector(t)
      up = Transform.get_up_vector(t)
      fwd = Transform.get_forward_vector(t)

      assert_in_delta Vec3.magnitude(right), 1.0, @tolerance
      assert_in_delta Vec3.magnitude(up), 1.0, @tolerance
      assert_in_delta Vec3.magnitude(fwd), 1.0, @tolerance
    end

    test "axis vectors are orthogonal" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 0.8)
      t = Transform.from_quaternion(q)

      right = Transform.get_right_vector(t)
      up = Transform.get_up_vector(t)
      fwd = Transform.get_forward_vector(t)

      assert_in_delta Vec3.dot(right, up), 0.0, @tolerance
      assert_in_delta Vec3.dot(up, fwd), 0.0, @tolerance
      assert_in_delta Vec3.dot(fwd, right), 0.0, @tolerance
    end

    test "right cross up equals forward" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 0.8)
      t = Transform.from_quaternion(q)

      right = Transform.get_right_vector(t)
      up = Transform.get_up_vector(t)
      fwd = Transform.get_forward_vector(t)
      computed_fwd = Vec3.cross(right, up)

      assert_in_delta Vec3.x(computed_fwd), Vec3.x(fwd), @tolerance
      assert_in_delta Vec3.y(computed_fwd), Vec3.y(fwd), @tolerance
      assert_in_delta Vec3.z(computed_fwd), Vec3.z(fwd), @tolerance
    end
  end

  describe "integration with existing Transform functions" do
    test "from_quaternion matches rotation_z for Z-axis rotation" do
      angle = :math.pi() / 4
      t1 = Transform.rotation_z(angle)
      q = Quaternion.from_axis_angle(Vec3.unit_z(), angle)
      t2 = Transform.from_quaternion(q)

      point = {1.0, 2.0, 3.0}
      {x1, y1, z1} = Transform.apply_to_point(t1, point)
      {x2, y2, z2} = Transform.apply_to_point(t2, point)

      assert_in_delta x1, x2, @tolerance
      assert_in_delta y1, y2, @tolerance
      assert_in_delta z1, z2, @tolerance
    end

    test "compose with from_quaternion" do
      rot = Transform.from_quaternion(Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 2))
      pos = Transform.translation(1.0, 0.0, 0.0)

      # compose(rot, pos) = rot * pos, so when applied to point p:
      # (rot * pos) * p = rot * (pos * p) - translation applied first, then rotation
      t = Transform.compose(rot, pos)
      {x, y, z} = Transform.apply_to_point(t, {0.0, 0.0, 0.0})

      # Point at origin: first translated to (1, 0, 0), then rotated 90° around Z to (0, 1, 0)
      assert_in_delta x, 0.0, @tolerance
      assert_in_delta y, 1.0, @tolerance
      assert_in_delta z, 0.0, @tolerance

      # A point at (1, 0, 0): first translated to (2, 0, 0), then rotated to (0, 2, 0)
      {x2, y2, z2} = Transform.apply_to_point(t, {1.0, 0.0, 0.0})
      assert_in_delta x2, 0.0, @tolerance
      assert_in_delta y2, 2.0, @tolerance
      assert_in_delta z2, 0.0, @tolerance
    end
  end
end
