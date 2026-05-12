# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ValidateLimitUnitsTransformerTest do
  use ExUnit.Case, async: true

  describe "revolute joints" do
    test "accepts angular units" do
      defmodule RevoluteAngular do
        use BB

        topology do
          link :base do
            joint :j do
              type :revolute

              limit do
                lower(~u(-90 degree))
                upper(~u(90 degree))
                effort(~u(10 newton_meter))
                velocity(~u(180 degree_per_second))
                acceleration(~u(360 degree_per_square_second))
              end

              link :child
            end
          end
        end
      end

      assert RevoluteAngular.robot()
    end

    test "rejects linear velocity on a revolute joint (schema-level rejection)" do
      # The schema already rejects this via `:or` validation before this
      # transformer runs. Test verifies that bad unit/joint combinations are
      # caught somewhere in the pipeline.
      assert_raise Spark.Error.DslError, fn ->
        defmodule RevoluteLinearVelocity do
          use BB

          topology do
            link :base do
              joint :j do
                type :revolute

                limit do
                  effort(~u(10 newton_meter))
                  velocity(~u(0.5 meter_per_second))
                end

                link :child
              end
            end
          end
        end
      end
    end

    test "rejects linear acceleration on a revolute joint" do
      assert_raise Spark.Error.DslError,
                   ~r/`meter_per_square_second`.*not compatible with a `revolute`/sm,
                   fn ->
                     defmodule RevoluteLinearAccel do
                       use BB

                       topology do
                         link :base do
                           joint :j do
                             type :revolute

                             limit do
                               effort(~u(10 newton_meter))
                               velocity(~u(180 degree_per_second))
                               acceleration(~u(1 meter_per_square_second))
                             end

                             link :child
                           end
                         end
                       end
                     end
                   end
    end
  end

  describe "prismatic joints" do
    test "accepts linear units" do
      defmodule PrismaticLinear do
        use BB

        topology do
          link :base do
            joint :j do
              type :prismatic

              limit do
                lower(~u(0 meter))
                upper(~u(1 meter))
                effort(~u(10 newton))
                velocity(~u(0.5 meter_per_second))
                acceleration(~u(2 meter_per_square_second))
              end

              link :child
            end
          end
        end
      end

      assert PrismaticLinear.robot()
    end

    test "rejects angular acceleration on a prismatic joint" do
      assert_raise Spark.Error.DslError,
                   ~r/`degree_per_square_second`.*not compatible with a `prismatic`/sm,
                   fn ->
                     defmodule PrismaticAngularAccel do
                       use BB

                       topology do
                         link :base do
                           joint :j do
                             type :prismatic

                             limit do
                               effort(~u(10 newton))
                               velocity(~u(0.5 meter_per_second))
                               acceleration(~u(360 degree_per_square_second))
                             end

                             link :child
                           end
                         end
                       end
                     end
                   end
    end
  end

  describe "fixed joints" do
    test "do not enforce unit compatibility (no limit block expected)" do
      defmodule FixedJoint do
        use BB

        topology do
          link :base do
            joint :j do
              type :fixed
              link :child
            end
          end
        end
      end

      assert FixedJoint.robot()
    end
  end
end
