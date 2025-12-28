# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.QuaternionTest do
  use ExUnit.Case, async: true
  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  @pi :math.pi()
  @tolerance 1.0e-6

  describe "new/4" do
    test "creates a normalised quaternion" do
      q = Quaternion.new(2, 0, 0, 0)
      assert_in_delta Quaternion.w(q), 1.0, @tolerance
    end

    test "preserves unit quaternion values" do
      q = Quaternion.new(0.707, 0, 0, 0.707)
      assert_in_delta Quaternion.w(q), 0.707, 0.01
      assert_in_delta Quaternion.z(q), 0.707, 0.01
    end
  end

  describe "identity/0" do
    test "returns identity quaternion" do
      q = Quaternion.identity()
      assert Quaternion.w(q) == 1.0
      assert Quaternion.x(q) == 0.0
      assert Quaternion.y(q) == 0.0
      assert Quaternion.z(q) == 0.0
    end

    test "identity rotation leaves vectors unchanged" do
      q = Quaternion.identity()
      v = Vec3.new(1.0, 2.0, 3.0)
      rotated = Quaternion.rotate_vector(q, v)

      assert_in_delta Vec3.x(rotated), 1.0, @tolerance
      assert_in_delta Vec3.y(rotated), 2.0, @tolerance
      assert_in_delta Vec3.z(rotated), 3.0, @tolerance
    end
  end

  describe "from_axis_angle/2" do
    test "creates quaternion for 90 degree rotation around Z" do
      q = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)

      assert_in_delta Quaternion.w(q), :math.cos(@pi / 4), @tolerance
      assert_in_delta Quaternion.x(q), 0.0, @tolerance
      assert_in_delta Quaternion.y(q), 0.0, @tolerance
      assert_in_delta Quaternion.z(q), :math.sin(@pi / 4), @tolerance
    end

    test "creates quaternion for 180 degree rotation around X" do
      q = Quaternion.from_axis_angle(Vec3.unit_x(), @pi)

      assert_in_delta Quaternion.w(q), 0.0, @tolerance
      assert_in_delta Quaternion.x(q), 1.0, @tolerance
      assert_in_delta Quaternion.y(q), 0.0, @tolerance
      assert_in_delta Quaternion.z(q), 0.0, @tolerance
    end

    test "zero angle gives identity" do
      q = Quaternion.from_axis_angle(Vec3.unit_x(), 0)

      assert_in_delta Quaternion.w(q), 1.0, @tolerance
      assert_in_delta Quaternion.x(q), 0.0, @tolerance
    end

    test "normalises non-unit axis" do
      q1 = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)
      q2 = Quaternion.from_axis_angle(Vec3.new(0, 0, 10), @pi / 2)

      assert_in_delta Quaternion.w(q1), Quaternion.w(q2), @tolerance
      assert_in_delta Quaternion.z(q1), Quaternion.z(q2), @tolerance
    end
  end

  describe "from_rotation_matrix/1" do
    test "identity matrix gives identity quaternion" do
      m = Nx.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
      q = Quaternion.from_rotation_matrix(m)

      assert_in_delta Quaternion.w(q), 1.0, @tolerance
      assert_in_delta Quaternion.x(q), 0.0, @tolerance
      assert_in_delta Quaternion.y(q), 0.0, @tolerance
      assert_in_delta Quaternion.z(q), 0.0, @tolerance
    end

    test "90 degree Z rotation matrix" do
      # Rotation matrix for 90 degrees around Z
      m = Nx.tensor([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
      q = Quaternion.from_rotation_matrix(m)

      expected = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)

      assert_in_delta Quaternion.w(q), Quaternion.w(expected), @tolerance
      assert_in_delta Quaternion.z(q), Quaternion.z(expected), @tolerance
    end

    test "round-trip: quaternion -> matrix -> quaternion" do
      original = Quaternion.from_axis_angle(Vec3.new(1, 1, 1), @pi / 3)
      matrix = Quaternion.to_rotation_matrix(original)
      recovered = Quaternion.from_rotation_matrix(matrix)

      # Quaternions can differ by sign but represent same rotation
      distance = Quaternion.angular_distance(original, recovered)
      assert_in_delta distance, 0.0, @tolerance
    end

    test "handles all Shepperd method branches" do
      # Trace > 0 (identity-like)
      m1 = Nx.tensor([[0.9, -0.1, 0.1], [0.1, 0.9, -0.1], [-0.1, 0.1, 0.9]])
      _q1 = Quaternion.from_rotation_matrix(m1)

      # m00 largest (X-dominant rotation)
      q_x = Quaternion.from_axis_angle(Vec3.unit_x(), 2.5)
      m_x = Quaternion.to_rotation_matrix(q_x)
      q_x_recovered = Quaternion.from_rotation_matrix(m_x)
      assert_in_delta Quaternion.angular_distance(q_x, q_x_recovered), 0.0, @tolerance

      # m11 largest (Y-dominant rotation)
      q_y = Quaternion.from_axis_angle(Vec3.unit_y(), 2.5)
      m_y = Quaternion.to_rotation_matrix(q_y)
      q_y_recovered = Quaternion.from_rotation_matrix(m_y)
      assert_in_delta Quaternion.angular_distance(q_y, q_y_recovered), 0.0, @tolerance

      # m22 largest (Z-dominant rotation)
      q_z = Quaternion.from_axis_angle(Vec3.unit_z(), 2.5)
      m_z = Quaternion.to_rotation_matrix(q_z)
      q_z_recovered = Quaternion.from_rotation_matrix(m_z)
      assert_in_delta Quaternion.angular_distance(q_z, q_z_recovered), 0.0, @tolerance
    end
  end

  describe "from_euler/4" do
    test "yaw only (rotation around Z)" do
      q = Quaternion.from_euler(0, 0, @pi / 2, :xyz)

      assert_in_delta Quaternion.w(q), :math.cos(@pi / 4), @tolerance
      assert_in_delta Quaternion.z(q), :math.sin(@pi / 4), @tolerance
    end

    test "pitch only (rotation around Y)" do
      q = Quaternion.from_euler(0, @pi / 2, 0, :xyz)

      assert_in_delta Quaternion.w(q), :math.cos(@pi / 4), @tolerance
      assert_in_delta Quaternion.y(q), :math.sin(@pi / 4), @tolerance
    end

    test "roll only (rotation around X)" do
      q = Quaternion.from_euler(@pi / 2, 0, 0, :xyz)

      assert_in_delta Quaternion.w(q), :math.cos(@pi / 4), @tolerance
      assert_in_delta Quaternion.x(q), :math.sin(@pi / 4), @tolerance
    end

    test "round-trip: euler -> quaternion -> euler preserves rotation" do
      roll = 0.3
      pitch = 0.2
      yaw = 0.5

      q1 = Quaternion.from_euler(roll, pitch, yaw, :xyz)
      {r, p, y} = Quaternion.to_euler(q1, :xyz)

      # Euler angles may differ due to multiple equivalent representations,
      # but the resulting rotation should be the same
      q2 = Quaternion.from_euler(r, p, y, :xyz)

      assert_in_delta Quaternion.angular_distance(q1, q2), 0.0, @tolerance
    end
  end

  describe "from_two_vectors/2" do
    test "X to Y gives 90 degree rotation around Z" do
      q = Quaternion.from_two_vectors(Vec3.unit_x(), Vec3.unit_y())
      rotated = Quaternion.rotate_vector(q, Vec3.unit_x())

      assert_in_delta Vec3.x(rotated), 0.0, @tolerance
      assert_in_delta Vec3.y(rotated), 1.0, @tolerance
      assert_in_delta Vec3.z(rotated), 0.0, @tolerance
    end

    test "Y to Z gives 90 degree rotation around X" do
      q = Quaternion.from_two_vectors(Vec3.unit_y(), Vec3.unit_z())
      rotated = Quaternion.rotate_vector(q, Vec3.unit_y())

      assert_in_delta Vec3.x(rotated), 0.0, @tolerance
      assert_in_delta Vec3.y(rotated), 0.0, @tolerance
      assert_in_delta Vec3.z(rotated), 1.0, @tolerance
    end

    test "Z to X gives 90 degree rotation around Y" do
      q = Quaternion.from_two_vectors(Vec3.unit_z(), Vec3.unit_x())
      rotated = Quaternion.rotate_vector(q, Vec3.unit_z())

      assert_in_delta Vec3.x(rotated), 1.0, @tolerance
      assert_in_delta Vec3.y(rotated), 0.0, @tolerance
      assert_in_delta Vec3.z(rotated), 0.0, @tolerance
    end

    test "parallel vectors give identity" do
      q = Quaternion.from_two_vectors(Vec3.unit_z(), Vec3.unit_z())

      assert_in_delta Quaternion.w(q), 1.0, @tolerance
      assert_in_delta Quaternion.x(q), 0.0, @tolerance
      assert_in_delta Quaternion.y(q), 0.0, @tolerance
      assert_in_delta Quaternion.z(q), 0.0, @tolerance
    end

    test "anti-parallel vectors give 180 degree rotation" do
      q = Quaternion.from_two_vectors(Vec3.unit_z(), Vec3.new(0, 0, -1))
      rotated = Quaternion.rotate_vector(q, Vec3.unit_z())

      assert_in_delta Vec3.x(rotated), 0.0, @tolerance
      assert_in_delta Vec3.y(rotated), 0.0, @tolerance
      assert_in_delta Vec3.z(rotated), -1.0, @tolerance
    end

    test "anti-parallel X vectors" do
      q = Quaternion.from_two_vectors(Vec3.unit_x(), Vec3.new(-1, 0, 0))
      rotated = Quaternion.rotate_vector(q, Vec3.unit_x())

      assert_in_delta Vec3.x(rotated), -1.0, @tolerance
      assert_in_delta Vec3.y(rotated), 0.0, @tolerance
      assert_in_delta Vec3.z(rotated), 0.0, @tolerance
    end

    test "arbitrary vectors" do
      from = Vec3.new(1, 0, 1)
      to = Vec3.new(0, 1, 0)

      q = Quaternion.from_two_vectors(from, to)
      rotated = Quaternion.rotate_vector(q, Vec3.normalise(from))

      # Should align with normalised `to`
      assert_in_delta Vec3.x(rotated), 0.0, @tolerance
      assert_in_delta Vec3.y(rotated), 1.0, @tolerance
      assert_in_delta Vec3.z(rotated), 0.0, @tolerance
    end

    test "normalises input vectors" do
      # Non-unit vectors should work the same as unit vectors
      q1 = Quaternion.from_two_vectors(Vec3.unit_x(), Vec3.unit_y())
      q2 = Quaternion.from_two_vectors(Vec3.new(10, 0, 0), Vec3.new(0, 5, 0))

      assert_in_delta Quaternion.angular_distance(q1, q2), 0.0, @tolerance
    end
  end

  describe "to_rotation_matrix/1" do
    test "identity quaternion gives identity matrix" do
      q = Quaternion.identity()
      m = Quaternion.to_rotation_matrix(q)

      assert_in_delta Nx.to_number(m[0][0]), 1.0, @tolerance
      assert_in_delta Nx.to_number(m[1][1]), 1.0, @tolerance
      assert_in_delta Nx.to_number(m[2][2]), 1.0, @tolerance
      assert_in_delta Nx.to_number(m[0][1]), 0.0, @tolerance
    end

    test "rotation matrix is orthogonal (det = 1)" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 1.5)
      m = Quaternion.to_rotation_matrix(q)

      # Check orthogonality: M * M^T = I
      mt = Nx.transpose(m)
      product = Nx.dot(m, mt)

      assert_in_delta Nx.to_number(product[0][0]), 1.0, @tolerance
      assert_in_delta Nx.to_number(product[1][1]), 1.0, @tolerance
      assert_in_delta Nx.to_number(product[2][2]), 1.0, @tolerance
      assert_in_delta Nx.to_number(product[0][1]), 0.0, @tolerance
    end
  end

  describe "to_axis_angle/1" do
    test "extracts axis and angle correctly" do
      axis = Vec3.unit_z()
      angle = @pi / 3
      q = Quaternion.from_axis_angle(axis, angle)

      {extracted_axis, extracted_angle} = Quaternion.to_axis_angle(q)

      assert_in_delta extracted_angle, angle, @tolerance
      assert_in_delta Vec3.z(extracted_axis), 1.0, @tolerance
    end

    test "identity quaternion gives zero angle" do
      q = Quaternion.identity()
      {_axis, angle} = Quaternion.to_axis_angle(q)

      assert_in_delta angle, 0.0, @tolerance
    end
  end

  describe "multiply/2" do
    test "identity is neutral element" do
      q = Quaternion.from_axis_angle(Vec3.unit_x(), @pi / 3)
      identity = Quaternion.identity()

      result = Quaternion.multiply(q, identity)

      assert_in_delta Quaternion.angular_distance(q, result), 0.0, @tolerance
    end

    test "composing rotations" do
      # Two 90-degree rotations around Z should give 180 degrees
      q = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)
      q2 = Quaternion.multiply(q, q)

      {_axis, angle} = Quaternion.to_axis_angle(q2)
      assert_in_delta angle, @pi, @tolerance
    end

    test "quaternion times inverse gives identity" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 1.0)
      qi = Quaternion.inverse(q)
      result = Quaternion.multiply(q, qi)

      assert_in_delta Quaternion.w(result), 1.0, @tolerance
      assert_in_delta Quaternion.x(result), 0.0, @tolerance
      assert_in_delta Quaternion.y(result), 0.0, @tolerance
      assert_in_delta Quaternion.z(result), 0.0, @tolerance
    end
  end

  describe "conjugate/1" do
    test "negates vector part" do
      q = Quaternion.new(0.5, 0.5, 0.5, 0.5)
      qc = Quaternion.conjugate(q)

      assert_in_delta Quaternion.w(qc), Quaternion.w(q), @tolerance
      assert_in_delta Quaternion.x(qc), -Quaternion.x(q), @tolerance
      assert_in_delta Quaternion.y(qc), -Quaternion.y(q), @tolerance
      assert_in_delta Quaternion.z(qc), -Quaternion.z(q), @tolerance
    end
  end

  describe "inverse/1" do
    test "inverse of unit quaternion equals conjugate" do
      q = Quaternion.from_axis_angle(Vec3.unit_x(), @pi / 4)
      qi = Quaternion.inverse(q)
      qc = Quaternion.conjugate(q)

      assert_in_delta Quaternion.w(qi), Quaternion.w(qc), @tolerance
      assert_in_delta Quaternion.x(qi), Quaternion.x(qc), @tolerance
    end
  end

  describe "rotate_vector/2" do
    test "90 degree rotation around Z rotates X to Y" do
      q = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)
      v = Vec3.unit_x()
      rotated = Quaternion.rotate_vector(q, v)

      assert_in_delta Vec3.x(rotated), 0.0, @tolerance
      assert_in_delta Vec3.y(rotated), 1.0, @tolerance
      assert_in_delta Vec3.z(rotated), 0.0, @tolerance
    end

    test "180 degree rotation around Z negates X and Y" do
      q = Quaternion.from_axis_angle(Vec3.unit_z(), @pi)
      v = Vec3.new(1, 1, 0)
      rotated = Quaternion.rotate_vector(q, v)

      assert_in_delta Vec3.x(rotated), -1.0, @tolerance
      assert_in_delta Vec3.y(rotated), -1.0, @tolerance
      assert_in_delta Vec3.z(rotated), 0.0, @tolerance
    end

    test "rotation preserves vector magnitude" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 1, 1), 1.234)
      v = Vec3.new(3.0, 4.0, 5.0)
      rotated = Quaternion.rotate_vector(q, v)

      original_mag = Vec3.magnitude(v)
      rotated_mag = Vec3.magnitude(rotated)

      assert_in_delta original_mag, rotated_mag, @tolerance
    end
  end

  describe "slerp/3" do
    test "t=0 returns first quaternion" do
      q1 = Quaternion.identity()
      q2 = Quaternion.from_axis_angle(Vec3.unit_z(), @pi)

      result = Quaternion.slerp(q1, q2, 0.0)

      assert_in_delta Quaternion.angular_distance(q1, result), 0.0, @tolerance
    end

    test "t=1 returns second quaternion" do
      q1 = Quaternion.identity()
      q2 = Quaternion.from_axis_angle(Vec3.unit_z(), @pi)

      result = Quaternion.slerp(q1, q2, 1.0)

      assert_in_delta Quaternion.angular_distance(q2, result), 0.0, @tolerance
    end

    test "t=0.5 returns halfway rotation" do
      q1 = Quaternion.identity()
      q2 = Quaternion.from_axis_angle(Vec3.unit_z(), @pi)

      result = Quaternion.slerp(q1, q2, 0.5)
      {_axis, angle} = Quaternion.to_axis_angle(result)

      assert_in_delta angle, @pi / 2, @tolerance
    end

    test "handles nearly identical quaternions" do
      q1 = Quaternion.identity()
      q2 = Quaternion.from_axis_angle(Vec3.unit_z(), 1.0e-8)

      result = Quaternion.slerp(q1, q2, 0.5)

      # Should not crash and should return something close to identity
      assert_in_delta Quaternion.w(result), 1.0, 0.01
    end
  end

  describe "angular_distance/2" do
    test "identical quaternions have zero distance" do
      q = Quaternion.from_axis_angle(Vec3.new(1, 2, 3), 1.0)

      assert_in_delta Quaternion.angular_distance(q, q), 0.0, @tolerance
    end

    test "opposite quaternions (q and -q) have zero distance" do
      q1 = Quaternion.new(0.5, 0.5, 0.5, 0.5)
      # -q represents same rotation
      q2 = %Quaternion{
        tensor: Nx.multiply(q1.tensor, -1)
      }

      assert_in_delta Quaternion.angular_distance(q1, q2), 0.0, @tolerance
    end

    test "90 degree rotation has distance of pi/2" do
      q1 = Quaternion.identity()
      q2 = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)

      assert_in_delta Quaternion.angular_distance(q1, q2), @pi / 2, @tolerance
    end

    test "180 degree rotation has distance of pi" do
      q1 = Quaternion.identity()
      q2 = Quaternion.from_axis_angle(Vec3.unit_z(), @pi)

      assert_in_delta Quaternion.angular_distance(q1, q2), @pi, @tolerance
    end
  end

  describe "list conversions" do
    test "to_list returns WXYZ order" do
      q = Quaternion.new(1, 2, 3, 4)
      list = Quaternion.to_list(q)

      # Normalised, so check relative values
      [w, x, y, z] = list
      assert w > 0
      assert x > 0
      assert y > 0
      assert z > 0
    end

    test "from_list round-trip" do
      original = [0.5, 0.5, 0.5, 0.5]
      q = Quaternion.from_list(original)
      recovered = Quaternion.to_list(q)

      Enum.zip(original, recovered)
      |> Enum.each(fn {o, r} -> assert_in_delta o, r, @tolerance end)
    end

    test "to_xyzw_list returns XYZW order" do
      q = Quaternion.identity()
      list = Quaternion.to_xyzw_list(q)

      assert list == [0.0, 0.0, 0.0, 1.0]
    end

    test "from_xyzw_list round-trip" do
      original = [0.0, 0.0, 0.707, 0.707]
      q = Quaternion.from_xyzw_list(original)
      recovered = Quaternion.to_xyzw_list(q)

      Enum.zip(original, recovered)
      |> Enum.each(fn {o, r} -> assert_in_delta o, r, 0.01 end)
    end
  end

  describe "tensor/1 and from_tensor/1" do
    test "tensor extraction and reconstruction" do
      q = Quaternion.from_axis_angle(Vec3.unit_z(), @pi / 2)
      t = Quaternion.tensor(q)

      assert Nx.shape(t) == {4}

      q2 = Quaternion.from_tensor(t)

      # Slightly relaxed tolerance due to double normalisation precision
      assert_in_delta Quaternion.angular_distance(q, q2), 0.0, 1.0e-3
    end
  end
end
