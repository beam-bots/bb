# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.TransformerTest do
  use ExUnit.Case, async: true
  alias Kinetix.Dsl.Info

  describe "auto-naming" do
    defmodule AutoNamedLinkRobot do
      @moduledoc false
      use Kinetix

      robot do
        link do
        end
      end
    end

    test "links without names get auto-named" do
      [link] = Info.robot(AutoNamedLinkRobot)
      assert link.name == :link_0
    end

    defmodule AutoNamedJointRobot do
      @moduledoc false
      use Kinetix

      robot do
        link :base do
          joint do
            type :fixed
            link :child
          end
        end
      end
    end

    test "joints without names get auto-named" do
      [link] = Info.robot(AutoNamedJointRobot)
      [joint] = link.joints
      assert joint.name == :joint_0
    end
  end

  describe "root link validation" do
    test "multiple root links produces error" do
      assert_raise Spark.Error.DslError, ~r/There can only be one link at the root/, fn ->
        defmodule MultipleRootLinksRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :link_a
            link :link_b
          end
        end
      end
    end
  end

  describe "joint child link validation" do
    test "joint without child link produces error" do
      assert_raise Spark.Error.DslError, ~r/All joints must connect to a child link/, fn ->
        defmodule JointWithoutChildRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              joint :orphan do
                type :fixed
              end
            end
          end
        end
      end
    end
  end

  describe "joint limit requirements" do
    test "revolute joint without limit produces error" do
      assert_raise Spark.Error.DslError, ~r/Limits must be present for revolute joints/, fn ->
        defmodule RevoluteWithoutLimitRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              joint :j1 do
                type :revolute
                link :child
              end
            end
          end
        end
      end
    end

    test "prismatic joint without limit produces error" do
      assert_raise Spark.Error.DslError, ~r/Limits must be present for prismatic joints/, fn ->
        defmodule PrismaticWithoutLimitRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              joint :j1 do
                type :prismatic
                link :child
              end
            end
          end
        end
      end
    end
  end

  describe "dynamics unit validation" do
    test "revolute joint with linear damping unit produces error" do
      assert_raise Spark.Error.DslError,
                   ~r/Expected unit.*to be compatible with newton meter/,
                   fn ->
                     defmodule RevoluteLinearDampingRobot do
                       @moduledoc false
                       use Kinetix

                       robot do
                         link :base do
                           joint :j1 do
                             type :revolute

                             dynamics do
                               damping ~u(1 newton_second_per_meter)
                             end

                             limit do
                               effort(~u(10 newton_meter))
                               velocity(~u(1 degree_per_second))
                             end

                             link :child
                           end
                         end
                       end
                     end
                   end
    end

    test "prismatic joint with angular damping unit produces error" do
      assert_raise Spark.Error.DslError,
                   ~r/Expected unit.*to be compatible with newton second per meter/,
                   fn ->
                     defmodule PrismaticAngularDampingRobot do
                       @moduledoc false
                       use Kinetix

                       robot do
                         link :base do
                           joint :j1 do
                             type :prismatic

                             dynamics do
                               damping ~u(1 newton_meter_second_per_degree)
                             end

                             limit do
                               effort(~u(10 newton_meter))
                               velocity(~u(1 meter_per_second))
                             end

                             link :child
                           end
                         end
                       end
                     end
                   end
    end

    test "fixed joint with dynamics produces error" do
      assert_raise Spark.Error.DslError,
                   ~r/Joint dynamics cannot be provided for fixed joints/,
                   fn ->
                     defmodule FixedWithDynamicsRobot do
                       @moduledoc false
                       use Kinetix

                       robot do
                         link :base do
                           joint :j1 do
                             type :fixed

                             dynamics do
                               damping ~u(1 newton_meter_second_per_degree)
                             end

                             link :child
                           end
                         end
                       end
                     end
                   end
    end
  end

  describe "limit unit validation" do
    test "revolute joint with linear limit produces error" do
      assert_raise Spark.Error.DslError, ~r/Expected unit.*to be compatible with degree/, fn ->
        defmodule RevoluteLinearLimitRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              joint :j1 do
                type :revolute

                limit do
                  lower(~u(0 meter))
                  upper(~u(1 meter))
                  effort(~u(10 newton_meter))
                  velocity(~u(1 degree_per_second))
                end

                link :child
              end
            end
          end
        end
      end
    end

    test "prismatic joint with angular limit produces error" do
      assert_raise Spark.Error.DslError, ~r/Expected unit.*to be compatible with meter/, fn ->
        defmodule PrismaticAngularLimitRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              joint :j1 do
                type :prismatic

                limit do
                  lower(~u(0 degree))
                  upper(~u(90 degree))
                  effort(~u(10 newton_meter))
                  velocity(~u(1 meter_per_second))
                end

                link :child
              end
            end
          end
        end
      end
    end
  end

  describe "colour validation" do
    test "colour values must be between 0 and 1" do
      assert_raise Spark.Error.DslError, ~r/Color value must be between 0 and 1/, fn ->
        defmodule InvalidColourRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              visual do
                box do
                  x ~u(0.1 meter)
                  y ~u(0.1 meter)
                  z ~u(0.1 meter)
                end

                material do
                  name :invalid

                  color do
                    red(1.5)
                    green(0.5)
                    blue(0.5)
                    alpha(1.0)
                  end
                end
              end
            end
          end
        end
      end
    end

    test "colour values must be numbers" do
      assert_raise Spark.Error.DslError, ~r/Expected a number for color value/, fn ->
        defmodule NonNumericColourRobot do
          @moduledoc false
          use Kinetix

          robot do
            link :base do
              visual do
                box do
                  x ~u(0.1 meter)
                  y ~u(0.1 meter)
                  z ~u(0.1 meter)
                end

                material do
                  name :invalid

                  color do
                    red("red")
                    green(0.5)
                    blue(0.5)
                    alpha(1.0)
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
