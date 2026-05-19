# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule BB.Igniter.TransmissionTest do
    use ExUnit.Case, async: true

    import Igniter.Test

    defmodule FakeDriver do
      @moduledoc false
    end

    defp run(source, opts \\ []) do
      igniter =
        test_project()
        |> Igniter.create_new_file("lib/my_robot.ex", source)
        |> BB.Igniter.Transmission.lift_reverse_question(FakeDriver, opts)

      source = Map.fetch!(igniter.rewrite.sources, "lib/my_robot.ex")
      Rewrite.Source.get(source, :content)
    end

    test "strips reverse?: true and inserts a transmission block with reversed? true" do
      output =
        run("""
        defmodule MyRobot do
          use BB

          topology do
            link :base do
              joint :shoulder do
                type :revolute

                limit do
                  lower ~u(-90 degree)
                  upper ~u(90 degree)
                  effort ~u(10 newton_meter)
                  velocity ~u(180 degree_per_second)
                end

                actuator :motor, {BB.Igniter.TransmissionTest.FakeDriver,
                  servo_id: 1, reverse?: true
                }

                link :arm
              end
            end
          end
        end
        """)

      assert output =~ "transmission do"
      assert output =~ "reversed?(true)"
      refute output =~ "reverse?:"
    end

    test "drops reverse?: false silently, no transmission block added" do
      output =
        run("""
        defmodule MyRobot do
          use BB

          topology do
            link :base do
              joint :shoulder do
                type :revolute

                actuator :motor, {BB.Igniter.TransmissionTest.FakeDriver,
                  servo_id: 1, reverse?: false
                }

                link :arm
              end
            end
          end
        end
        """)

      refute output =~ "reverse?:"
      refute output =~ "transmission do"
    end

    test "with lift_offset?: true, computes offset from asymmetric limits" do
      output =
        run(
          """
          defmodule MyRobot do
            use BB

            topology do
              link :base do
                joint :shoulder do
                  type :revolute

                  limit do
                    lower ~u(-10 degree)
                    upper ~u(190 degree)
                    effort ~u(10 newton_meter)
                    velocity ~u(180 degree_per_second)
                  end

                  actuator :motor, {BB.Igniter.TransmissionTest.FakeDriver,
                    servo_id: 1, reverse?: true
                  }

                  link :arm
                end
              end
            end
          end
          """,
          lift_offset?: true
        )

      assert output =~ "transmission do"
      assert output =~ "offset(~u(90.0 degree))"
      assert output =~ "reversed?(true)"
      refute output =~ "reverse?:"
    end

    test "with lift_offset?: true, no offset for symmetric limits" do
      output =
        run(
          """
          defmodule MyRobot do
            use BB

            topology do
              link :base do
                joint :shoulder do
                  type :revolute

                  limit do
                    lower ~u(-90 degree)
                    upper ~u(90 degree)
                    effort ~u(10 newton_meter)
                    velocity ~u(180 degree_per_second)
                  end

                  actuator :motor, {BB.Igniter.TransmissionTest.FakeDriver,
                    servo_id: 1, reverse?: true
                  }

                  link :arm
                end
              end
            end
          end
          """,
          lift_offset?: true
        )

      assert output =~ "transmission do"
      assert output =~ "reversed?(true)"
      refute output =~ "offset("
    end

    test "leaves untouched joints with no matching actuator" do
      input = """
      defmodule MyRobot do
        use BB

        topology do
          link :base do
            joint :shoulder do
              type :revolute

              actuator :motor, {SomeOther.Driver, opts: 1}

              link :arm
            end
          end
        end
      end
      """

      output = run(input)

      refute output =~ "transmission do"
    end
  end
end
