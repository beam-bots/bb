# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.EstimatorTest do
  use ExUnit.Case, async: false

  alias BB.Dsl.Estimator, as: EstimatorEntity
  alias BB.Dsl.Info
  alias BB.Dsl.Link
  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message
  alias BB.Message.Sensor.Imu

  describe "DSL - sensor-nested" do
    defmodule SensorNestedRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          sensor :imu, MySensor do
            estimator :orientation, EchoEstimator
          end
        end
      end
    end

    test "estimator parses as a child of its sensor" do
      [link] = Info.topology(SensorNestedRobot) |> Enum.filter(&is_struct(&1, Link))
      [sensor] = link.sensors
      [%EstimatorEntity{} = est] = sensor.estimators

      assert est.name == :orientation
      assert est.child_spec == EchoEstimator
      assert est.inputs == []
      assert est.outputs == []
    end
  end

  describe "DSL - link-nested" do
    defmodule LinkNestedRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          sensor :imu, MySensor
          sensor :odom, MySensor

          estimator :pose, MultiInputEstimator do
            input :imu, [:sensor, :base_link, :imu], driver?: true
            input :odom, [:sensor, :base_link, :odom]
            sync_tolerance(~u(20 millisecond))
          end
        end
      end
    end

    test "estimator parses as a child of its link with declared inputs" do
      [link] = Info.topology(LinkNestedRobot) |> Enum.filter(&is_struct(&1, Link))
      [%EstimatorEntity{} = est] = link.estimators

      assert est.name == :pose
      assert est.child_spec == MultiInputEstimator
      assert length(est.inputs) == 2

      [imu_input, odom_input] = est.inputs
      assert imu_input.name == :imu
      assert imu_input.driver?
      assert odom_input.name == :odom
      refute odom_input.driver?

      assert est.sync_tolerance != nil
    end
  end

  describe "Verifier - sensor-nested invariants" do
    test "DSL prevents declaring `input` blocks inside a sensor-nested estimator" do
      assert_raise CompileError, fn ->
        Code.eval_string("""
        defmodule BB.EstimatorTest.BadSensorNested do
          use BB

          topology do
            link :base_link do
              sensor :imu, MySensor do
                estimator :orientation, EchoEstimator do
                  input :foo, [:sensor, :base_link, :imu]
                end
              end
            end
          end
        end
        """)
      end
    end
  end

  describe "Verifier - link-nested invariants" do
    test "rejects link-nested estimator without inputs" do
      errors =
        collect_verifier_errors(fn ->
          defmodule NoInputs do
            @moduledoc false
            use BB

            topology do
              link :base_link do
                estimator :orphan, EchoEstimator
              end
            end
          end
        end)

      assert_error_matches(errors, ~r/must declare at least one `input`/)
    end

    test "rejects link-nested estimator with multiple drivers" do
      errors =
        collect_verifier_errors(fn ->
          defmodule TwoDrivers do
            @moduledoc false
            use BB

            topology do
              link :base_link do
                sensor :imu, MySensor
                sensor :odom, MySensor

                estimator :pose, MultiInputEstimator do
                  input :imu, [:sensor, :base_link, :imu], driver?: true
                  input :odom, [:sensor, :base_link, :odom], driver?: true
                end
              end
            end
          end
        end)

      assert_error_matches(errors, ~r/multiple driver inputs/)
    end

    test "rejects link-nested multi-input estimator with no driver" do
      errors =
        collect_verifier_errors(fn ->
          defmodule NoDriver do
            @moduledoc false
            use BB

            topology do
              link :base_link do
                sensor :imu, MySensor
                sensor :odom, MySensor

                estimator :pose, MultiInputEstimator do
                  input :imu, [:sensor, :base_link, :imu]
                  input :odom, [:sensor, :base_link, :odom]
                end
              end
            end
          end
        end)

      assert_error_matches(errors, ~r/no driver/)
    end

    test "rejects input referencing an unknown path" do
      errors =
        collect_verifier_errors(fn ->
          defmodule UnknownInput do
            @moduledoc false
            use BB

            topology do
              link :base_link do
                estimator :pose, EchoEstimator do
                  input :imu, [:sensor, :base_link, :imu_does_not_exist]
                end
              end
            end
          end
        end)

      assert_error_matches(errors, ~r/unknown path/)
    end

    test "rejects sync_tolerance on single-input estimator" do
      errors =
        collect_verifier_errors(fn ->
          defmodule SyncOnSingle do
            @moduledoc false
            use BB

            topology do
              link :base_link do
                sensor :imu, MySensor

                estimator :pose, EchoEstimator do
                  input :imu, [:sensor, :base_link, :imu]
                  sync_tolerance(~u(20 millisecond))
                end
              end
            end
          end
        end)

      assert_error_matches(errors, ~r/single input but declares `sync_tolerance`/)
    end

    test "rejects estimator that consumes its own output (self-cycle)" do
      errors =
        collect_verifier_errors(fn ->
          defmodule SelfCycle do
            @moduledoc false
            use BB

            topology do
              link :base_link do
                estimator :loop, EchoEstimator do
                  input :me, [:estimator, :base_link, :loop]
                end
              end
            end
          end
        end)

      assert_error_matches(errors, ~r/self-cycle/)
    end
  end

  defp collect_verifier_errors(fun) do
    Process.put({Spark.Dsl, :test_collector}, self())
    fun.()

    receive do
      {Spark.Dsl, :verifier_errors, _module, errors} -> errors
    after
      100 -> []
    end
  after
    Process.delete({Spark.Dsl, :test_collector})
  end

  defp assert_error_matches(errors, regex) do
    assert Enum.any?(errors, fn
             %Spark.Error.DslError{message: msg} -> msg =~ regex
             %{message: msg} when is_binary(msg) -> msg =~ regex
             _ -> false
           end),
           "Expected an error matching #{inspect(regex)}, got: #{inspect(errors)}"
  end

  describe "Runtime - sensor-nested single-input dispatch" do
    defmodule EchoRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          sensor :imu, MySensor do
            estimator :orientation, EchoEstimator
          end
        end
      end
    end

    test "publishes its input back on the estimator's own path" do
      start_supervised!(EchoRobot)

      out_path = [:sensor, :base_link, :imu, :orientation]
      in_path = [:sensor, :base_link, :imu]

      {:ok, _} = BB.subscribe(EchoRobot, out_path)
      {:ok, msg} = build_imu_message()
      BB.publish(EchoRobot, in_path, msg)

      assert_receive {:bb, ^out_path, %Message{payload: %Imu{}}}, 500
    end
  end

  describe "Runtime - link-nested multi-input fan-in" do
    defmodule FanInRobot do
      @moduledoc false
      use BB

      sensors do
        sensor :imu, MySensor
        sensor :odom, MySensor
      end

      topology do
        link :base_link do
          estimator :pose, MultiInputEstimator do
            input :imu, [:sensor, :imu], driver?: true
            input :odom, [:sensor, :odom]
            sync_tolerance(~u(50 millisecond))
          end
        end
      end
    end

    setup do
      :persistent_term.put(:estimator_test_pid, self())
      on_exit(fn -> :persistent_term.erase(:estimator_test_pid) end)
      start_supervised!({FanInRobot, []})
      :ok
    end

    test "dispatches when both inputs are within sync_tolerance" do
      {:ok, odom_msg} = build_imu_message()
      BB.publish(FanInRobot, [:sensor, :odom], odom_msg)

      {:ok, imu_msg} = build_imu_message()
      BB.publish(FanInRobot, [:sensor, :imu], imu_msg)

      assert_receive {:multi_input, %{imu: %Message{}, odom: %Message{}}}, 500
    end

    test "drops dispatch when non-driver is older than sync_tolerance" do
      handler_id = "dropped-#{:erlang.unique_integer([:positive])}"
      test_pid = self()

      :telemetry.attach(
        handler_id,
        [:bb, :estimator, :dropped],
        fn _event, _meas, metadata, _ -> send(test_pid, {:dropped, metadata}) end,
        nil
      )

      try do
        {:ok, odom_msg} = build_imu_message_with_offset(-1_000_000_000)
        BB.publish(FanInRobot, [:sensor, :odom], odom_msg)

        {:ok, imu_msg} = build_imu_message()
        BB.publish(FanInRobot, [:sensor, :imu], imu_msg)

        assert_receive {:dropped, %{reason: :sync_miss}}, 500
        refute_receive {:multi_input, _}, 100
      after
        :telemetry.detach(handler_id)
      end
    end
  end

  defp build_imu_message do
    Imu.new(:test,
      orientation: Quaternion.identity(),
      angular_velocity: Vec3.zero(),
      linear_acceleration: Vec3.zero()
    )
  end

  defp build_imu_message_with_offset(offset_ns) do
    {:ok, msg} = build_imu_message()
    {:ok, %{msg | monotonic_time: msg.monotonic_time + offset_ns}}
  end
end
