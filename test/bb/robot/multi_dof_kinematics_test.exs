# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.MultiDofKinematicsTest do
  @moduledoc """
  Kinematics for `:planar` and `:floating` joints.

  Correctness here is established two ways, both deliberately independent of the
  code under test:

  - **Forward kinematics against hand-computed poses.** A round-trip through our
    own composition is self-consistent — a systematically wrong transform order
    passes it perfectly — so expected poses are worked out by hand from the
    fixture geometry.
  - **The Jacobian against central finite differences of forward kinematics.**
    Numerically differentiating FK is an independent derivation, so it catches
    column ordering, sign, and per-DoF-width errors. Note this validates the
    Jacobian *given* FK, which is why FK needs its own check above.
  """
  use ExUnit.Case, async: true

  alias BB.ExampleRobots.DifferentialDriveRobot
  alias BB.ExampleRobots.FloatingDrone
  alias BB.ExampleRobots.PlanarRover
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Robot.Kinematics

  @half_pi :math.pi() / 2

  defp rover, do: PlanarRover.robot()
  defp drone, do: FloatingDrone.robot()

  defp position(robot, configurations, link) do
    Kinematics.link_position(robot, configurations, link)
  end

  defp assert_position({x, y, z}, {ex, ey, ez}) do
    assert_in_delta x, ex, 1.0e-9
    assert_in_delta y, ey, 1.0e-9
    assert_in_delta z, ez, 1.0e-9
  end

  describe "planar joint forward kinematics" do
    # The mast sits 0.2 m above the chassis, so with the base at the origin the
    # sensor head is directly above it.
    test "identity configuration leaves the chain at its origin offsets" do
      configurations = %{base: Transform2D.identity(), mast: 0.0}

      assert_position(position(rover(), configurations, :chassis), {0.0, 0.0, 0.0})
      assert_position(position(rover(), configurations, :sensor_head), {0.0, 0.0, 0.2})
    end

    # Hand-computed: the base translates by (12.4, -3.1) in the XY plane, and the
    # mast offset is along the plane normal so yaw cannot move it.
    test "in-plane translation carries the whole chain" do
      configurations = %{base: Transform2D.new(12.4, -3.1, 0.0), mast: 0.0}

      assert_position(position(rover(), configurations, :chassis), {12.4, -3.1, 0.0})
      assert_position(position(rover(), configurations, :sensor_head), {12.4, -3.1, 0.2})
    end

    test "yaw about the plane normal does not move a point on the axis" do
      configurations = %{base: Transform2D.new(0.0, 0.0, 1.0), mast: 0.0}

      assert_position(position(rover(), configurations, :sensor_head), {0.0, 0.0, 0.2})
    end

    # Rotating the base by +90 degrees about Z then translating (1, 0) in the
    # *rotated* frame is a different pose from translating first — the base's own
    # translation is applied in the parent frame, so this checks the convention.
    test "translation is in the parent frame, not the rotated one" do
      configurations = %{base: Transform2D.new(1.0, 0.0, @half_pi), mast: 0.0}

      assert_position(position(rover(), configurations, :sensor_head), {1.0, 0.0, 0.2})
    end

    test "the mast composes on top of the base" do
      configurations = %{base: Transform2D.new(2.0, 3.0, 0.0), mast: @half_pi}

      # The mast rotates about Z and the sensor head is on that axis, so its
      # position is unchanged while its orientation is not.
      assert_position(position(rover(), configurations, :sensor_head), {2.0, 3.0, 0.2})

      transform = Kinematics.forward_kinematics(rover(), configurations, :sensor_head)
      rotated = Transform.apply_to_point(transform, Vec3.unit_x())

      assert_position({Vec3.x(rotated), Vec3.y(rotated), Vec3.z(rotated)}, {2.0, 4.0, 0.2})
    end

    test "agrees with composing the joint transforms by hand" do
      configurations = %{base: Transform2D.new(1.5, -2.5, 0.6), mast: 0.4}

      expected =
        Transform.compose(
          Kinematics.compute_joint_transform(rover(), configurations, :base),
          Kinematics.compute_joint_transform(rover(), configurations, :mast)
        )

      actual = Kinematics.forward_kinematics(rover(), configurations, :sensor_head)

      assert_transforms_equal(actual, expected)
    end

    test "all_link_transforms/2 agrees with forward_kinematics/3" do
      configurations = %{base: Transform2D.new(1.5, -2.5, 0.6), mast: 0.4}
      transforms = Kinematics.all_link_transforms(rover(), configurations)

      for link <- [:odom, :chassis, :sensor_head] do
        assert_transforms_equal(
          Map.fetch!(transforms, link),
          Kinematics.forward_kinematics(rover(), configurations, link)
        )
      end
    end
  end

  describe "planar joints with a non-Z surface normal" do
    # `axis` is the plane's surface normal, so a rover declared with a Y normal
    # moves in the XZ plane. Assuming XY would put the sensor head in the wrong
    # place, and nothing else in this suite would notice.
    test "the plane follows the joint's axis" do
      robot = tilted_rover()
      {u, v} = Transform2D.plane_basis(Vec3.unit_y())

      # A unit of the base's x moves the chassis one metre along the plane's
      # first axis, and a unit of its y along the second — whatever those are.
      assert_position(
        position(robot, %{base: Transform2D.new(1.0, 0.0, 0.0), mast: 0.0}, :chassis),
        {Vec3.x(u), Vec3.y(u), Vec3.z(u)}
      )

      assert_position(
        position(robot, %{base: Transform2D.new(0.0, 1.0, 0.0), mast: 0.0}, :chassis),
        {Vec3.x(v), Vec3.y(v), Vec3.z(v)}
      )
    end

    test "the plane is not the XY plane" do
      robot = tilted_rover()

      # A Y normal means motion in XZ, so the chassis must leave the XY plane.
      {_x, y, z} = position(robot, %{base: Transform2D.new(0.0, 1.0, 0.0), mast: 0.0}, :chassis)

      assert_in_delta y, 0.0, 1.0e-9
      refute_in_delta z, 0.0, 1.0e-9
    end

    test "yaw is about the declared normal, leaving it fixed" do
      robot = tilted_rover()
      configurations = %{base: Transform2D.new(0.0, 0.0, 0.7), mast: 0.0}

      transform = Kinematics.forward_kinematics(robot, configurations, :chassis)
      fixed = Transform.apply_to_point(transform, Vec3.unit_y())

      assert_position({Vec3.x(fixed), Vec3.y(fixed), Vec3.z(fixed)}, {0.0, 1.0, 0.0})
    end
  end

  describe "floating joint forward kinematics" do
    test "identity configuration leaves the chain at its origin offsets" do
      configurations = %{base: Transform.identity(), gimbal: 0.0}

      assert_position(position(drone(), configurations, :airframe), {0.0, 0.0, 0.0})
      assert_position(position(drone(), configurations, :camera), {0.1, 0.0, -0.05})
    end

    test "the stored transform is used verbatim" do
      pose = Transform.translation(Vec3.new(5.0, -2.0, 30.0))
      configurations = %{base: pose, gimbal: 0.0}

      assert_position(position(drone(), configurations, :airframe), {5.0, -2.0, 30.0})
      assert_position(position(drone(), configurations, :camera), {5.1, -2.0, 29.95})
    end

    # Hand-computed: yaw of +90 degrees takes the gimbal's local +x offset to
    # world +y, and leaves the -z offset alone.
    test "a rotated base carries the child link's offset with it" do
      configurations = %{base: Transform.rotation_z(@half_pi), gimbal: 0.0}

      assert_position(position(drone(), configurations, :camera), {0.0, 0.1, -0.05})
    end

    test "a full pose composes rotation then translation in the parent frame" do
      pose =
        Transform.compose(
          Transform.translation(Vec3.new(1.0, 2.0, 3.0)),
          Transform.rotation_z(@half_pi)
        )

      configurations = %{base: pose, gimbal: 0.0}

      assert_position(position(drone(), configurations, :camera), {1.0, 2.1, 2.95})
    end

    # A floating base and a revolute joint do not commute, so composing them the
    # wrong way round must be detectable rather than coincidentally equal.
    test "composition with the gimbal does not commute" do
      pose =
        Transform.compose(
          Transform.translation(Vec3.new(1.0, 2.0, 3.0)),
          Transform.rotation_z(0.9)
        )

      forwards = %{base: pose, gimbal: 0.7}

      correct = Kinematics.forward_kinematics(drone(), forwards, :camera)

      swapped =
        Transform.compose(
          Kinematics.compute_joint_transform(drone(), forwards, :gimbal),
          Kinematics.compute_joint_transform(drone(), forwards, :base)
        )

      refute Nx.to_binary(Transform.tensor(correct)) == Nx.to_binary(Transform.tensor(swapped))
    end

    test "forward kinematics is bit-exact for the stored pose" do
      pose =
        Transform.compose(
          Transform.translation(Vec3.new(12.400000000000001, -3.0999999999999996, 0.7)),
          Transform.from_quaternion(
            Quaternion.from_axis_angle(
              Vec3.normalise(Vec3.new(0.3, -0.7, 0.64)),
              1.2345678901234567
            )
          )
        )

      # The airframe's parent joint has no origin, so its base-frame transform
      # must be the stored pose with no arithmetic applied at all.
      actual = Kinematics.forward_kinematics(drone(), %{base: pose, gimbal: 0.0}, :airframe)

      assert Nx.to_binary(Transform.tensor(actual)) == Nx.to_binary(Transform.tensor(pose))
    end

    test "all_link_transforms/2 agrees with forward_kinematics/3" do
      pose =
        Transform.compose(
          Transform.translation(Vec3.new(1.0, 2.0, 3.0)),
          Transform.rotation_z(0.9)
        )

      configurations = %{base: pose, gimbal: 0.7}
      transforms = Kinematics.all_link_transforms(drone(), configurations)

      for link <- [:world, :airframe, :camera] do
        assert_transforms_equal(
          Map.fetch!(transforms, link),
          Kinematics.forward_kinematics(drone(), configurations, link)
        )
      end
    end
  end

  describe "jacobian_columns/2" do
    test "reports three columns for a planar joint and one for a revolute" do
      assert Kinematics.jacobian_columns(rover(), [:base, :mast]) == [
               {:base, 0},
               {:base, 1},
               {:base, 2},
               {:mast, 0}
             ]
    end

    test "reports six columns for a floating joint" do
      assert Kinematics.jacobian_columns(drone(), [:base, :gimbal]) == [
               {:base, 0},
               {:base, 1},
               {:base, 2},
               {:base, 3},
               {:base, 4},
               {:base, 5},
               {:gimbal, 0}
             ]
    end

    test "a fixed joint contributes no columns" do
      assert Kinematics.jacobian_columns(DifferentialDriveRobot.robot(), [:caster_joint]) == []
    end
  end

  describe "jacobian width" do
    test "is the sum of degrees of freedom, not the joint count" do
      configurations = %{base: Transform2D.new(1.0, 2.0, 0.3), mast: 0.4}

      jacobian =
        Kinematics.position_jacobian(rover(), configurations, :sensor_head, [:base, :mast])

      assert Nx.shape(jacobian) == {3, 4}
    end

    test "a floating base contributes three translation and three rotation columns" do
      configurations = %{base: Transform.rotation_z(0.4), gimbal: 0.2}

      jacobian = Kinematics.jacobian(drone(), configurations, :camera, [:base, :gimbal])

      assert Nx.shape(jacobian) == {6, 7}
    end
  end

  describe "jacobian against finite differences" do
    test "planar base and revolute mast" do
      configurations = %{base: Transform2D.new(1.5, -2.5, 0.6), mast: 0.4}

      assert_jacobian_matches_finite_differences(
        rover(),
        configurations,
        :sensor_head,
        [:base, :mast]
      )
    end

    test "planar base with a non-Z surface normal" do
      configurations = %{base: Transform2D.new(1.5, -2.5, 0.6), mast: 0.4}

      assert_jacobian_matches_finite_differences(
        tilted_rover(),
        configurations,
        :sensor_head,
        [:base, :mast]
      )
    end

    test "floating base and revolute gimbal" do
      pose =
        Transform.compose(
          Transform.translation(Vec3.new(1.0, 2.0, 3.0)),
          Transform.from_quaternion(
            Quaternion.from_axis_angle(Vec3.normalise(Vec3.new(0.3, -0.7, 0.64)), 0.85)
          )
        )

      configurations = %{base: pose, gimbal: 0.4}

      assert_jacobian_matches_finite_differences(
        drone(),
        configurations,
        :camera,
        [:base, :gimbal]
      )
    end
  end

  # Perturbs each degree of freedom by the *true* rigid motion it represents —
  # a real translation or a real `from_axis_angle` rotation, composed into the
  # joint's stored configuration — and differentiates forward kinematics. This is
  # an independent derivation of the Jacobian, so agreement is meaningful.
  defp assert_jacobian_matches_finite_differences(robot, configurations, target, joint_names) do
    columns = Kinematics.jacobian_columns(robot, joint_names)
    analytic = Kinematics.position_jacobian(robot, configurations, target, joint_names)
    h = 1.0e-7

    for {{joint_name, dof}, column} <- Enum.with_index(columns) do
      plus = position(robot, perturb(robot, configurations, joint_name, dof, h), target)
      minus = position(robot, perturb(robot, configurations, joint_name, dof, -h), target)

      for {axis, row} <- Enum.with_index([:x, :y, :z]) do
        assert_in_delta Nx.to_number(analytic[row][column]),
                        (elem(plus, row) - elem(minus, row)) / (2 * h),
                        1.0e-5,
                        "#{inspect(joint_name)} dof #{dof}, #{axis} row"
      end
    end
  end

  defp perturb(robot, configurations, joint_name, dof, amount) do
    {:ok, joint} = BB.Robot.get_joint(robot, joint_name)

    Map.put(
      configurations,
      joint_name,
      perturbed_configuration(joint, Map.fetch!(configurations, joint_name), dof, amount)
    )
  end

  # The perturbation is local to the joint, so it composes on the right — the same
  # side `I + hat(delta)` sits on in the kernel.
  defp perturbed_configuration(%{type: :planar}, configuration, dof, amount) do
    delta =
      case dof do
        0 -> Transform2D.new(amount, 0.0, 0.0)
        1 -> Transform2D.new(0.0, amount, 0.0)
        2 -> Transform2D.new(0.0, 0.0, amount)
      end

    Transform2D.compose(configuration, delta)
  end

  defp perturbed_configuration(%{type: :floating}, configuration, dof, amount) do
    delta =
      case dof do
        0 -> Transform.translation(Vec3.new(amount, 0.0, 0.0))
        1 -> Transform.translation(Vec3.new(0.0, amount, 0.0))
        2 -> Transform.translation(Vec3.new(0.0, 0.0, amount))
        3 -> Transform.from_axis_angle(Vec3.unit_x(), amount)
        4 -> Transform.from_axis_angle(Vec3.unit_y(), amount)
        5 -> Transform.from_axis_angle(Vec3.unit_z(), amount)
      end

    Transform.compose(configuration, delta)
  end

  defp perturbed_configuration(_joint, configuration, 0, amount), do: configuration + amount

  defp assert_transforms_equal(actual, expected) do
    Enum.zip(
      Nx.to_flat_list(Transform.tensor(actual)),
      Nx.to_flat_list(Transform.tensor(expected))
    )
    |> Enum.each(fn {left, right} -> assert_in_delta left, right, 1.0e-9 end)
  end

  defmodule TiltedRover do
    @moduledoc """
    A rover whose ground plane has a **Y** surface normal rather than Z.

    Exists so the tests prove `axis` is actually respected, rather than the plane
    being assumed to be XY.
    """
    use BB
    import BB.Unit

    topology do
      link :odom do
        joint :base do
          type(:planar)

          axis do
            roll(~u(-90 degree))
          end

          link :chassis do
            joint :mast do
              type(:revolute)

              origin do
                x(~u(0.2 meter))
              end

              axis do
              end

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(90 degree_per_second))
              end

              link(:sensor_head)
            end
          end
        end
      end
    end
  end

  defp tilted_rover, do: TiltedRover.robot()
end
