# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.JointTest do
  use ExUnit.Case, async: true
  alias BB.Dsl.{Axis, Dynamics, Info, Joint, Limit, Origin}
  import BB.Unit

  describe "revolute joint" do
    defmodule RevoluteRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :revolute

            origin do
              x ~u(0.1 meter)
              yaw ~u(45 degree)
            end

            axis do
            end

            limit do
              lower(~u(-90 degree))
              upper(~u(90 degree))
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "revolute joint with required limit compiles" do
      assert [link] = Info.topology(RevoluteRobot)
      assert [joint] = link.joints
      assert is_struct(joint, Joint)
      assert joint.name == :joint1
      assert joint.type == :revolute
    end

    test "revolute joint has origin" do
      [link] = Info.topology(RevoluteRobot)
      [joint] = link.joints
      assert is_struct(joint.origin, Origin)
      assert joint.origin.x == ~u(0.1 meter)
      assert joint.origin.yaw == ~u(45 degree)
    end

    test "revolute joint has axis with default Z orientation" do
      [link] = Info.topology(RevoluteRobot)
      [joint] = link.joints
      assert is_struct(joint.axis, Axis)
      assert joint.axis.roll == ~u(0 degree)
      assert joint.axis.pitch == ~u(0 degree)
      assert joint.axis.yaw == ~u(0 degree)
    end

    test "revolute joint has limit with degree units" do
      [link] = Info.topology(RevoluteRobot)
      [joint] = link.joints
      assert is_struct(joint.limit, Limit)
      assert joint.limit.lower == ~u(-90 degree)
      assert joint.limit.upper == ~u(90 degree)
      assert joint.limit.effort == ~u(10 newton_meter)
      assert joint.limit.velocity == ~u(100 degree_per_second)
    end

    test "revolute joint connects to child link" do
      [link] = Info.topology(RevoluteRobot)
      [joint] = link.joints
      assert joint.link.name == :child_link
    end

    defmodule RevoluteWithDynamicsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :revolute

            dynamics do
              damping ~u(0.5 newton_meter_second_per_degree)
              friction ~u(0.1 newton_meter)
            end

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "revolute joint with dynamics compiles" do
      [link] = Info.topology(RevoluteWithDynamicsRobot)
      [joint] = link.joints
      assert is_struct(joint.dynamics, Dynamics)
      assert joint.dynamics.damping == ~u(0.5 newton_meter_second_per_degree)
      assert joint.dynamics.friction == ~u(0.1 newton_meter)
    end
  end

  describe "continuous joint" do
    defmodule ContinuousRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :wheel_joint do
            type :continuous

            axis do
            end

            link :wheel do
            end
          end
        end
      end
    end

    test "continuous joint compiles without limit" do
      [link] = Info.topology(ContinuousRobot)
      [joint] = link.joints
      assert joint.type == :continuous
      assert joint.name == :wheel_joint
      assert is_nil(joint.limit)
    end

    defmodule ContinuousWithLimitRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :continuous

            limit do
              effort(~u(5 newton_meter))
              velocity(~u(360 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "continuous joint with optional limit compiles" do
      [link] = Info.topology(ContinuousWithLimitRobot)
      [joint] = link.joints
      assert is_struct(joint.limit, Limit)
      assert joint.limit.effort == ~u(5 newton_meter)
    end

    defmodule ContinuousWithDynamicsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :continuous

            dynamics do
              damping ~u(0.2 newton_meter_second_per_degree)
              friction ~u(0.05 newton_meter)
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "continuous joint with dynamics compiles" do
      [link] = Info.topology(ContinuousWithDynamicsRobot)
      [joint] = link.joints
      assert is_struct(joint.dynamics, Dynamics)
    end
  end

  describe "prismatic joint" do
    defmodule PrismaticRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :slider do
            type :prismatic

            axis do
            end

            limit do
              lower(~u(0 meter))
              upper(~u(0.5 meter))
              effort(~u(100 newton))
              velocity(~u(0.1 meter_per_second))
            end

            link :sliding_link do
            end
          end
        end
      end
    end

    test "prismatic joint with required limit compiles" do
      [link] = Info.topology(PrismaticRobot)
      [joint] = link.joints
      assert joint.type == :prismatic
      assert joint.name == :slider
    end

    test "prismatic limit uses meter units" do
      [link] = Info.topology(PrismaticRobot)
      [joint] = link.joints
      assert joint.limit.lower == ~u(0 meter)
      assert joint.limit.upper == ~u(0.5 meter)
      assert joint.limit.velocity == ~u(0.1 meter_per_second)
    end

    defmodule PrismaticWithDynamicsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :slider do
            type :prismatic

            dynamics do
              damping ~u(1.0 newton_second_per_meter)
              friction ~u(0.5 newton)
            end

            limit do
              effort(~u(100 newton))
              velocity(~u(0.1 meter_per_second))
            end

            link :sliding_link do
            end
          end
        end
      end
    end

    test "prismatic dynamics uses linear units" do
      [link] = Info.topology(PrismaticWithDynamicsRobot)
      [joint] = link.joints
      assert joint.dynamics.damping == ~u(1.0 newton_second_per_meter)
      assert joint.dynamics.friction == ~u(0.5 newton)
    end
  end

  describe "fixed joint" do
    defmodule FixedRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :fixed_joint do
            type :fixed

            origin do
              x ~u(0.1 meter)
              y ~u(0.2 meter)
            end

            link :attached_link do
            end
          end
        end
      end
    end

    test "fixed joint compiles with minimal config" do
      [link] = Info.topology(FixedRobot)
      [joint] = link.joints
      assert joint.type == :fixed
      assert joint.name == :fixed_joint
      assert is_nil(joint.limit)
      assert is_nil(joint.dynamics)
    end

    test "fixed joint can have origin" do
      [link] = Info.topology(FixedRobot)
      [joint] = link.joints
      assert joint.origin.x == ~u(0.1 meter)
      assert joint.origin.y == ~u(0.2 meter)
    end
  end

  describe "floating joint" do
    defmodule FloatingRobot do
      @moduledoc false
      use BB

      topology do
        link :world do
          joint :floating_base do
            type :floating

            link :robot_base do
            end
          end
        end
      end
    end

    test "floating joint compiles" do
      [link] = Info.topology(FloatingRobot)
      [joint] = link.joints
      assert joint.type == :floating
      assert joint.name == :floating_base
      assert is_nil(joint.limit)
      assert is_nil(joint.dynamics)
    end
  end

  describe "planar joint" do
    defmodule PlanarRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :planar_joint do
            type :planar

            axis do
            end

            link :sliding_link do
            end
          end
        end
      end
    end

    test "planar joint compiles" do
      [link] = Info.topology(PlanarRobot)
      [joint] = link.joints
      assert joint.type == :planar
      assert joint.name == :planar_joint
    end

    test "planar joint with optional axis compiles" do
      [link] = Info.topology(PlanarRobot)
      [joint] = link.joints
      assert is_struct(joint.axis, Axis)
      assert joint.axis.roll == ~u(0 degree)
    end

    defmodule PlanarWithDynamicsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :planar_joint do
            type :planar

            dynamics do
              damping ~u(0.5 newton_second_per_meter)
              friction ~u(0.1 newton)
            end

            link :sliding_link do
            end
          end
        end
      end
    end

    test "planar dynamics uses linear units" do
      [link] = Info.topology(PlanarWithDynamicsRobot)
      [joint] = link.joints
      assert joint.dynamics.damping == ~u(0.5 newton_second_per_meter)
      assert joint.dynamics.friction == ~u(0.1 newton)
    end
  end

  describe "joint auto-naming" do
    defmodule AutoNamedJointRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint do
            type :fixed

            link :child1 do
            end
          end

          joint do
            type :fixed

            link :child2 do
            end
          end
        end
      end
    end

    test "joints auto-generate names when not provided" do
      [link] = Info.topology(AutoNamedJointRobot)
      names = Enum.map(link.joints, & &1.name)
      assert :joint_0 in names
      assert :joint_1 in names
    end
  end

  describe "origin entity" do
    defmodule OriginRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :fixed

            origin do
              x ~u(1 meter)
              y ~u(2 meter)
              z ~u(3 meter)
              roll ~u(10 degree)
              pitch ~u(20 degree)
              yaw ~u(30 degree)
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "origin with all translation values" do
      [link] = Info.topology(OriginRobot)
      [joint] = link.joints
      assert joint.origin.x == ~u(1 meter)
      assert joint.origin.y == ~u(2 meter)
      assert joint.origin.z == ~u(3 meter)
    end

    test "origin with all rotation values" do
      [link] = Info.topology(OriginRobot)
      [joint] = link.joints
      assert joint.origin.roll == ~u(10 degree)
      assert joint.origin.pitch == ~u(20 degree)
      assert joint.origin.yaw == ~u(30 degree)
    end

    defmodule OriginDefaultsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :fixed

            origin do
              x ~u(1 meter)
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "origin values default to zero when omitted" do
      [link] = Info.topology(OriginDefaultsRobot)
      [joint] = link.joints
      assert joint.origin.x == ~u(1 meter)
      assert joint.origin.y == ~u(0 meter)
      assert joint.origin.z == ~u(0 meter)
      assert joint.origin.roll == ~u(0 degree)
      assert joint.origin.pitch == ~u(0 degree)
      assert joint.origin.yaw == ~u(0 degree)
    end

    defmodule OriginAlternativeUnitsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :fixed

            origin do
              x ~u(100 centimeter)
              roll ~u(1 radian)
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "origin accepts various length units" do
      [link] = Info.topology(OriginAlternativeUnitsRobot)
      [joint] = link.joints
      assert joint.origin.x == ~u(100 centimeter)
    end

    test "origin accepts various angle units" do
      [link] = Info.topology(OriginAlternativeUnitsRobot)
      [joint] = link.joints
      assert joint.origin.roll == ~u(1 radian)
    end
  end

  describe "axis entity" do
    defmodule AxisRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :revolute

            axis do
              roll ~u(10 degree)
              pitch ~u(20 degree)
              yaw ~u(30 degree)
            end

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "axis with roll, pitch, yaw components" do
      [link] = Info.topology(AxisRobot)
      [joint] = link.joints
      assert joint.axis.roll == ~u(10 degree)
      assert joint.axis.pitch == ~u(20 degree)
      assert joint.axis.yaw == ~u(30 degree)
    end

    defmodule AxisDefaultsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :revolute

            axis do
            end

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "axis values default to zero (Z axis orientation)" do
      [link] = Info.topology(AxisDefaultsRobot)
      [joint] = link.joints
      assert joint.axis.roll == ~u(0 degree)
      assert joint.axis.pitch == ~u(0 degree)
      assert joint.axis.yaw == ~u(0 degree)
    end

    defmodule YAxisRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :revolute

            axis do
              roll ~u(-90 degree)
            end

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "axis with roll rotates default Z to Y direction" do
      [link] = Info.topology(YAxisRobot)
      [joint] = link.joints
      assert joint.axis.roll == ~u(-90 degree)
      assert joint.axis.pitch == ~u(0 degree)
      assert joint.axis.yaw == ~u(0 degree)
    end
  end

  describe "limit entity" do
    defmodule LimitOptionalBoundsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :continuous

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :child_link do
            end
          end
        end
      end
    end

    test "limit lower/upper are optional" do
      [link] = Info.topology(LimitOptionalBoundsRobot)
      [joint] = link.joints
      assert is_nil(joint.limit.lower)
      assert is_nil(joint.limit.upper)
      assert joint.limit.effort == ~u(10 newton_meter)
      assert joint.limit.velocity == ~u(100 degree_per_second)
    end
  end
end
