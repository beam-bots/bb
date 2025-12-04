# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Urdf.ExporterTest do
  use ExUnit.Case, async: true
  import Kinetix.Unit

  alias Kinetix.Urdf.Exporter

  defmodule MinimalRobot do
    use Kinetix

    topology do
      link(:base)
    end
  end

  defmodule RobotWithInertial do
    use Kinetix

    topology do
      link :base do
        inertial do
          mass(~u(5 kilogram))

          inertia do
            ixx(~u(0.1 kilogram_square_meter))
            iyy(~u(0.2 kilogram_square_meter))
            izz(~u(0.3 kilogram_square_meter))
            ixy(~u(0.01 kilogram_square_meter))
            ixz(~u(0.02 kilogram_square_meter))
            iyz(~u(0.03 kilogram_square_meter))
          end

          origin do
            x(~u(0.1 meter))
            y(~u(0.2 meter))
            z(~u(0.3 meter))
          end
        end
      end
    end
  end

  defmodule RobotWithVisual do
    use Kinetix

    topology do
      link :base do
        visual do
          origin do
            x(~u(0.1 meter))
          end

          box do
            x(~u(1 meter))
            y(~u(2 meter))
            z(~u(3 meter))
          end

          material do
            name(:red)

            color do
              red(1.0)
              green(0.0)
              blue(0.0)
              alpha(1.0)
            end
          end
        end
      end
    end
  end

  defmodule RobotWithCollision do
    use Kinetix

    topology do
      link :base do
        collision do
          name(:col1)

          sphere do
            radius(~u(0.5 meter))
          end
        end

        collision do
          name(:col2)

          cylinder do
            radius(~u(0.1 meter))
            height(~u(1 meter))
          end
        end
      end
    end
  end

  defmodule RobotWithJoints do
    use Kinetix

    topology do
      link :base do
        joint :shoulder do
          type(:revolute)

          origin do
            z(~u(10 centimeter))
          end

          axis do
          end

          limit do
            lower(~u(-90 degree))
            upper(~u(90 degree))
            effort(~u(50 newton_meter))
            velocity(~u(2 degree_per_second))
          end

          dynamics do
            damping(~u(0.5 newton_meter_second_per_radian))
            friction(~u(0.1 newton_meter))
          end

          link(:upper_arm)
        end
      end
    end
  end

  defmodule RobotWithFixedJoint do
    use Kinetix

    topology do
      link :base do
        joint :fixed_joint do
          type(:fixed)

          origin do
            z(~u(5 centimeter))
          end

          link(:attachment)
        end
      end
    end
  end

  defmodule RobotWithContinuousJoint do
    use Kinetix

    topology do
      link :base do
        joint :wheel do
          type(:continuous)

          axis do
            roll(~u(-90 degree))
          end

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link(:wheel_link)
        end
      end
    end
  end

  describe "export/1" do
    test "exports minimal robot" do
      {:ok, xml} = Exporter.export(MinimalRobot)

      assert xml =~ ~s(<robot name="Kinetix.Urdf.ExporterTest.MinimalRobot">)
      assert xml =~ ~s(<link name="base"/>)
    end

    test "returns error for non-existent module" do
      assert {:error, {:module_not_found, NonExistent, _}} = Exporter.export(NonExistent)
    end

    test "returns error for module without robot/0" do
      assert {:error, {:not_a_kinetix_module, Enum}} = Exporter.export(Enum)
    end
  end

  describe "export_robot/1 with inertial properties" do
    test "exports mass" do
      {:ok, xml} = Exporter.export(RobotWithInertial)

      assert xml =~ ~s(<mass value="5"/>)
    end

    test "exports inertia tensor" do
      {:ok, xml} = Exporter.export(RobotWithInertial)

      assert xml =~ ~s(ixx="0.1")
      assert xml =~ ~s(iyy="0.2")
      assert xml =~ ~s(izz="0.3")
      assert xml =~ ~s(ixy="0.01")
      assert xml =~ ~s(ixz="0.02")
      assert xml =~ ~s(iyz="0.03")
    end

    test "exports center of mass origin" do
      {:ok, xml} = Exporter.export(RobotWithInertial)

      assert xml =~ ~s(<inertial>)
      assert xml =~ ~s(xyz="0.1 0.2 0.3")
    end
  end

  describe "export_robot/1 with visual" do
    test "exports visual geometry" do
      {:ok, xml} = Exporter.export(RobotWithVisual)

      assert xml =~ ~s(<visual>)
      assert xml =~ ~s(<geometry>)
      assert xml =~ ~s(<box size="1 2 3"/>)
    end

    test "exports visual origin" do
      {:ok, xml} = Exporter.export(RobotWithVisual)

      assert xml =~ ~s(xyz="0.1 0 0")
    end

    test "exports material with color" do
      {:ok, xml} = Exporter.export(RobotWithVisual)

      assert xml =~ ~s(<material name="red">)
      assert xml =~ ~s(<color rgba="1 0 0 1"/>)
    end
  end

  describe "export_robot/1 with collision" do
    test "exports collision elements" do
      {:ok, xml} = Exporter.export(RobotWithCollision)

      assert xml =~ ~s(<collision name="col1">)
      assert xml =~ ~s(<collision name="col2">)
    end

    test "exports sphere geometry" do
      {:ok, xml} = Exporter.export(RobotWithCollision)

      assert xml =~ ~s(<sphere radius="0.5"/>)
    end

    test "exports cylinder geometry with length attribute" do
      {:ok, xml} = Exporter.export(RobotWithCollision)

      assert xml =~ ~s(<cylinder radius="0.1" length="1"/>)
    end
  end

  describe "export_robot/1 with joints" do
    test "exports joint with type" do
      {:ok, xml} = Exporter.export(RobotWithJoints)

      assert xml =~ ~s(<joint name="shoulder" type="revolute">)
    end

    test "exports joint origin" do
      {:ok, xml} = Exporter.export(RobotWithJoints)

      assert xml =~ ~s(xyz="0 0 0.1")
      assert xml =~ ~s(rpy="0 0 0")
    end

    test "exports parent and child links" do
      {:ok, xml} = Exporter.export(RobotWithJoints)

      assert xml =~ ~s(<parent link="base"/>)
      assert xml =~ ~s(<child link="upper_arm"/>)
    end

    test "exports axis" do
      {:ok, xml} = Exporter.export(RobotWithJoints)

      assert xml =~ ~s(<axis xyz="0 0 1"/>)
    end

    test "exports limits with position bounds" do
      {:ok, xml} = Exporter.export(RobotWithJoints)

      assert xml =~ ~r/lower="-1\.57079\d*"/
      assert xml =~ ~r/upper="1\.57079\d*"/
      assert xml =~ ~s(effort="50")
      assert xml =~ ~r/velocity="0\.03490\d*"/
    end

    test "exports dynamics" do
      {:ok, xml} = Exporter.export(RobotWithJoints)

      assert xml =~ ~s(<dynamics damping="0.5" friction="0.1"/>)
    end
  end

  describe "export_robot/1 with fixed joint" do
    test "omits axis for fixed joints" do
      {:ok, xml} = Exporter.export(RobotWithFixedJoint)

      refute xml =~ ~s(<axis)
    end

    test "omits limits for fixed joints" do
      {:ok, xml} = Exporter.export(RobotWithFixedJoint)

      refute xml =~ ~s(<limit)
    end

    test "omits dynamics for fixed joints" do
      {:ok, xml} = Exporter.export(RobotWithFixedJoint)

      refute xml =~ ~s(<dynamics)
    end
  end

  describe "export_robot/1 with continuous joint" do
    test "omits position limits for continuous joints" do
      {:ok, xml} = Exporter.export(RobotWithContinuousJoint)

      refute xml =~ ~s(lower=)
      refute xml =~ ~s(upper=)
    end

    test "includes effort and velocity limits" do
      {:ok, xml} = Exporter.export(RobotWithContinuousJoint)

      assert xml =~ ~s(effort="10")
      assert xml =~ ~r/velocity="1\.74532\d*"/
    end
  end

  describe "XML structure" do
    test "includes XML declaration" do
      {:ok, xml} = Exporter.export(MinimalRobot)

      assert String.starts_with?(xml, "<?xml version=\"1.0\"?>")
    end

    test "root element is robot" do
      {:ok, xml} = Exporter.export(MinimalRobot)

      assert xml =~ ~r/<robot name="[^"]+">.*<\/robot>/s
    end
  end
end
