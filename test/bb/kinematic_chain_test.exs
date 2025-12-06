# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.KinematicChainTest do
  use ExUnit.Case, async: true
  alias BB.Dsl.{Info, Joint, Link}
  import BB.Unit

  describe "two-link chain" do
    defmodule TwoLinkRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :joint1 do
            type :revolute

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(1 degree_per_second))
            end

            link :link1
          end
        end
      end
    end

    test "root link has one joint" do
      [root] = Info.topology(TwoLinkRobot)
      assert root.name == :base_link
      assert length(root.joints) == 1
    end

    test "joint connects to child link" do
      [root] = Info.topology(TwoLinkRobot)
      [joint] = root.joints
      assert joint.name == :joint1
      assert is_struct(joint.link, Link)
      assert joint.link.name == :link1
    end
  end

  describe "three-link serial chain" do
    defmodule ThreeLinkSerialRobot do
      @moduledoc false
      use BB

      topology do
        link :base do
          joint :shoulder do
            type :revolute

            limit do
              effort(~u(50 newton_meter))
              velocity(~u(2 degree_per_second))
            end

            link :upper_arm do
              joint :elbow do
                type :revolute

                limit do
                  effort(~u(30 newton_meter))
                  velocity(~u(3 degree_per_second))
                end

                link :forearm
              end
            end
          end
        end
      end
    end

    test "can traverse full chain" do
      [base] = Info.topology(ThreeLinkSerialRobot)
      assert base.name == :base

      [shoulder] = base.joints
      assert shoulder.name == :shoulder

      upper_arm = shoulder.link
      assert upper_arm.name == :upper_arm

      [elbow] = upper_arm.joints
      assert elbow.name == :elbow

      forearm = elbow.link
      assert forearm.name == :forearm
      assert forearm.joints == []
    end

    test "all links are Link structs" do
      [base] = Info.topology(ThreeLinkSerialRobot)

      assert is_struct(base, Link)
      assert is_struct(base.joints |> hd() |> Map.get(:link), Link)

      assert is_struct(
               base.joints
               |> hd()
               |> Map.get(:link)
               |> Map.get(:joints)
               |> hd()
               |> Map.get(:link),
               Link
             )
    end

    test "all joints are Joint structs" do
      [base] = Info.topology(ThreeLinkSerialRobot)

      [shoulder] = base.joints
      assert is_struct(shoulder, Joint)

      [elbow] = shoulder.link.joints
      assert is_struct(elbow, Joint)
    end
  end

  describe "branching kinematic tree" do
    defmodule BranchingRobot do
      @moduledoc false
      use BB

      topology do
        link :torso do
          joint :left_shoulder do
            type :revolute

            limit do
              effort(~u(40 newton_meter))
              velocity(~u(2 degree_per_second))
            end

            link :left_arm
          end

          joint :right_shoulder do
            type :revolute

            limit do
              effort(~u(40 newton_meter))
              velocity(~u(2 degree_per_second))
            end

            link :right_arm
          end

          joint :neck do
            type :revolute

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(1 degree_per_second))
            end

            link :head
          end
        end
      end
    end

    test "single link can have multiple joints" do
      [torso] = Info.topology(BranchingRobot)
      assert length(torso.joints) == 3
    end

    test "all branches are accessible" do
      [torso] = Info.topology(BranchingRobot)
      joint_names = Enum.map(torso.joints, & &1.name)
      assert :left_shoulder in joint_names
      assert :right_shoulder in joint_names
      assert :neck in joint_names
    end

    test "each branch has its own child link" do
      [torso] = Info.topology(BranchingRobot)
      child_names = Enum.map(torso.joints, & &1.link.name)
      assert :left_arm in child_names
      assert :right_arm in child_names
      assert :head in child_names
    end
  end

  describe "mixed joint types in chain" do
    defmodule MixedJointRobot do
      @moduledoc false
      use BB

      topology do
        link :base do
          joint :prismatic_joint do
            type :prismatic

            limit do
              lower(~u(0 meter))
              upper(~u(1 meter))
              effort(~u(100 newton))
              velocity(~u(0.5 meter_per_second))
            end

            link :sliding_platform do
              joint :revolute_joint do
                type :revolute

                limit do
                  lower(~u(-90 degree))
                  upper(~u(90 degree))
                  effort(~u(50 newton_meter))
                  velocity(~u(1 degree_per_second))
                end

                link :rotating_arm do
                  joint :fixed_joint do
                    type :fixed
                    link :end_effector
                  end
                end
              end
            end
          end
        end
      end
    end

    test "chain can mix joint types" do
      [base] = Info.topology(MixedJointRobot)

      [prismatic] = base.joints
      assert prismatic.type == :prismatic

      [revolute] = prismatic.link.joints
      assert revolute.type == :revolute

      [fixed] = revolute.link.joints
      assert fixed.type == :fixed
    end

    test "different joint types have appropriate properties" do
      [base] = Info.topology(MixedJointRobot)

      [prismatic] = base.joints
      assert prismatic.limit.lower == ~u(0 meter)
      assert prismatic.limit.upper == ~u(1 meter)

      [revolute] = prismatic.link.joints
      assert revolute.limit.lower == ~u(-90 degree)
      assert revolute.limit.upper == ~u(90 degree)

      [fixed] = revolute.link.joints
      assert is_nil(fixed.limit)
    end
  end

  describe "deep chain with properties" do
    defmodule SixDofArmRobot do
      @moduledoc false
      use BB

      topology do
        link :base do
          inertial do
            mass(~u(10 kilogram))
          end

          joint :j1 do
            type :revolute

            origin do
              z ~u(0.1 meter)
            end

            limit do
              effort(~u(100 newton_meter))
              velocity(~u(2 degree_per_second))
            end

            link :link1 do
              inertial do
                mass(~u(5 kilogram))
              end

              joint :j2 do
                type :revolute

                origin do
                  z ~u(0.5 meter)
                end

                limit do
                  effort(~u(80 newton_meter))
                  velocity(~u(2 degree_per_second))
                end

                link :link2 do
                  joint :j3 do
                    type :revolute

                    origin do
                      z ~u(0.4 meter)
                    end

                    limit do
                      effort(~u(60 newton_meter))
                      velocity(~u(3 degree_per_second))
                    end

                    link :link3 do
                      joint :j4 do
                        type :revolute

                        limit do
                          effort(~u(40 newton_meter))
                          velocity(~u(4 degree_per_second))
                        end

                        link :link4 do
                          joint :j5 do
                            type :revolute

                            limit do
                              effort(~u(20 newton_meter))
                              velocity(~u(5 degree_per_second))
                            end

                            link :link5 do
                              joint :j6 do
                                type :revolute

                                limit do
                                  effort(~u(10 newton_meter))
                                  velocity(~u(6 degree_per_second))
                                end

                                link :end_effector
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end

    test "six joint chain compiles" do
      [base] = Info.topology(SixDofArmRobot)
      assert base.name == :base
    end

    test "can traverse all six joints" do
      [base] = Info.topology(SixDofArmRobot)

      joints =
        base
        |> collect_joints([])
        |> Enum.reverse()

      assert length(joints) == 6
      assert Enum.map(joints, & &1.name) == [:j1, :j2, :j3, :j4, :j5, :j6]
    end

    test "properties are preserved through chain" do
      [base] = Info.topology(SixDofArmRobot)
      assert base.inertial.mass == ~u(10 kilogram)

      link1 = base.joints |> hd() |> Map.get(:link)
      assert link1.inertial.mass == ~u(5 kilogram)
    end

    test "origins are set on joints" do
      [base] = Info.topology(SixDofArmRobot)

      [j1] = base.joints
      assert j1.origin.z == ~u(0.1 meter)

      [j2] = j1.link.joints
      assert j2.origin.z == ~u(0.5 meter)
    end
  end

  defp collect_joints(%Link{joints: []}, acc), do: acc

  defp collect_joints(%Link{joints: joints}, acc) do
    Enum.reduce(joints, acc, fn joint, acc ->
      collect_joints(joint.link, [joint | acc])
    end)
  end
end
