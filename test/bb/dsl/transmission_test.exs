# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.TransmissionTest do
  use ExUnit.Case, async: true

  describe "literal values" do
    test "joint without a transmission block has nil transmission on the robot" do
      defmodule NoTransmission do
        use BB

        topology do
          link :base do
            joint :shoulder do
              type :revolute

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(180 degree_per_second))
              end

              link :arm
            end
          end
        end
      end

      joint = BB.Robot.get_joint(NoTransmission.robot(), :shoulder)
      assert joint.transmission == nil
    end

    test "transmission block converts to SI floats on the optimised robot" do
      defmodule Literal do
        use BB

        topology do
          link :base do
            joint :shoulder do
              type :revolute

              transmission do
                reduction 50.0
                offset(~u(45 degree))
                reversed? true
              end

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(180 degree_per_second))
              end

              link :arm
            end
          end
        end
      end

      transmission = BB.Robot.get_joint(Literal.robot(), :shoulder).transmission
      assert transmission.reduction == 50.0
      assert_in_delta transmission.offset, :math.pi() / 4, 1.0e-9
      assert transmission.reversed? == true
    end

    test "prismatic joint converts offset to metres" do
      defmodule Prismatic do
        use BB

        topology do
          link :base do
            joint :slider do
              type :prismatic

              transmission do
                reduction 2.0
                offset(~u(10 millimeter))
              end

              limit do
                effort(~u(5 newton))
                velocity(~u(0.1 meter_per_second))
              end

              link :child
            end
          end
        end
      end

      transmission = BB.Robot.get_joint(Prismatic.robot(), :slider).transmission
      assert_in_delta transmission.offset, 0.01, 1.0e-9
      assert transmission.reversed? == false
      assert transmission.reduction == 2.0
    end

    test "defaults applied when only some fields are given" do
      defmodule PartialDefaults do
        use BB

        topology do
          link :base do
            joint :shoulder do
              type :revolute

              transmission do
                reversed? true
              end

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(180 degree_per_second))
              end

              link :arm
            end
          end
        end
      end

      transmission = BB.Robot.get_joint(PartialDefaults.robot(), :shoulder).transmission
      assert transmission.reduction == 1.0
      assert transmission.offset == 0.0
      assert transmission.reversed? == true
    end
  end

  describe "joint-type unit validation" do
    test "rejects a metre offset on a revolute joint" do
      assert_raise Spark.Error.DslError, ~r/not compatible with a `revolute`/sm, fn ->
        defmodule BadOffset do
          use BB

          topology do
            link :base do
              joint :shoulder do
                type :revolute

                transmission do
                  offset(~u(10 millimeter))
                end

                limit do
                  effort(~u(10 newton_meter))
                  velocity(~u(180 degree_per_second))
                end

                link :arm
              end
            end
          end
        end
      end
    end

    test "rejects a degree offset on a prismatic joint" do
      assert_raise Spark.Error.DslError, ~r/not compatible with a `prismatic`/sm, fn ->
        defmodule BadOffsetPrismatic do
          use BB

          topology do
            link :base do
              joint :slider do
                type :prismatic

                transmission do
                  offset(~u(10 degree))
                end

                limit do
                  effort(~u(5 newton))
                  velocity(~u(0.1 meter_per_second))
                end

                link :child
              end
            end
          end
        end
      end
    end
  end

  describe "parameter references" do
    test "reduction, offset, and reversed? all accept param/1" do
      defmodule Parameterised do
        use BB

        parameters do
          group :tx do
            param :reduction, type: :float, default: 50.0
            param :offset, type: {:unit, :degree}, default: ~u(45 degree)
            param :reversed?, type: :boolean, default: true
          end
        end

        topology do
          link :base do
            joint :shoulder do
              type :revolute

              transmission do
                reduction(param([:tx, :reduction]))
                offset(param([:tx, :offset]))
                reversed?(param([:tx, :reversed?]))
              end

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(180 degree_per_second))
              end

              link :arm
            end
          end
        end
      end

      robot = Parameterised.robot()
      transmission = BB.Robot.get_joint(robot, :shoulder).transmission

      assert transmission.reduction == nil
      assert transmission.offset == nil
      assert transmission.reversed? == nil

      subs = robot.param_subscriptions

      assert Map.has_key?(subs, [:tx, :reduction])
      assert Map.has_key?(subs, [:tx, :offset])
      assert Map.has_key?(subs, [:tx, :reversed?])
    end

  end
end
