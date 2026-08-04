# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.JointAxisValidationTest do
  @moduledoc """
  A joint's `axis` means something different for each type, and for two of them it
  means nothing at all. `BB.Dsl.TopologyTransformer` owns the checks, alongside
  the existing "limits are required for revolute and prismatic joints" rule.
  """
  use ExUnit.Case, async: true

  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot.Kinematics

  describe "planar joints" do
    test "accept an axis, which is the plane's surface normal" do
      defmodule PlanarWithAxis do
        use BB

        topology do
          link :odom do
            joint :base do
              type(:planar)

              axis do
              end

              link(:chassis)
            end
          end
        end
      end

      {:ok, joint} = BB.Robot.get_joint(PlanarWithAxis.robot(), :base)
      assert joint.axis == {0.0, 0.0, 1.0}
    end

    # Without a normal there is no plane, so defaulting one would silently pick a
    # plane that has nothing to do with the robot.
    test "are rejected without an axis" do
      error =
        assert_raise Spark.Error.DslError, fn ->
          defmodule PlanarWithoutAxis do
            use BB

            topology do
              link :odom do
                joint :base do
                  type(:planar)

                  link(:chassis)
                end
              end
            end
          end
        end

      message = Exception.message(error)

      assert message =~ "An axis must be present for planar joints"
      assert message =~ "surface normal"
    end
  end

  describe "floating joints" do
    test "are accepted without an axis" do
      defmodule FloatingWithoutAxis do
        use BB

        topology do
          link :world do
            joint :base do
              type(:floating)

              link(:airframe)
            end
          end
        end
      end

      assert FloatingWithoutAxis.robot()
    end

    # Six degrees of freedom leave no distinguished direction, so accepting an
    # axis would let a user believe they had constrained something.
    test "are rejected with an axis" do
      error =
        assert_raise Spark.Error.DslError, fn ->
          defmodule FloatingWithAxis do
            use BB

            topology do
              link :world do
                joint :base do
                  type(:floating)

                  axis do
                  end

                  link(:airframe)
                end
              end
            end
          end
        end

      message = Exception.message(error)

      assert message =~ "Cannot set an axis when parent joint is floating"
      # The message should point at the type the user probably wanted.
      assert message =~ ":planar"
    end
  end

  describe "fixed joints" do
    test "are rejected with an axis, and say why" do
      error =
        assert_raise Spark.Error.DslError, fn ->
          defmodule FixedWithAxis do
            use BB

            topology do
              link :base do
                joint :j do
                  type(:fixed)

                  axis do
                  end

                  link(:child)
                end
              end
            end
          end
        end

      # This rule already existed but built its error with `module:` where
      # `message:` was meant, so the diagnostic read `nil`.
      assert Exception.message(error) =~ "Cannot set an axis when parent joint is fixed"
    end
  end

  describe "single-DoF joints" do
    test "may still omit their axis entirely" do
      defmodule RevoluteWithoutAxis do
        use BB
        import BB.Unit

        topology do
          link :base do
            joint :j do
              type(:revolute)

              limit do
                effort(~u(10 newton_meter))
                velocity(~u(90 degree_per_second))
              end

              link(:child)
            end
          end
        end
      end

      robot = RevoluteWithoutAxis.robot()
      {:ok, joint} = BB.Robot.get_joint(robot, :j)

      # An omitted axis stays `nil` on the built joint; the Z default is applied
      # by `BB.Robot.Kinematics` at use time, not baked in by the builder. So a
      # rotation about it must still behave as a rotation about Z.
      assert joint.axis == nil

      rotated =
        robot
        |> Kinematics.forward_kinematics(%{j: :math.pi() / 2}, :child)
        |> Transform.apply_to_point(Vec3.unit_x())

      assert Enum.map(Vec3.to_list(rotated), &Float.round(&1, 9)) == [0.0, 1.0, 0.0]
    end
  end

  describe "nested joints" do
    test "are checked below the root, not just at it" do
      assert_raise Spark.Error.DslError, ~r/floating/, fn ->
        defmodule NestedFloatingWithAxis do
          use BB
          import BB.Unit

          topology do
            link :base do
              joint :shallow do
                type(:revolute)

                limit do
                  effort(~u(10 newton_meter))
                  velocity(~u(90 degree_per_second))
                end

                link :middle do
                  joint :deep do
                    type(:floating)

                    axis do
                    end

                    link(:tip)
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
