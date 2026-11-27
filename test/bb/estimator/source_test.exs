# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.SourceTest.FakeSource do
  @moduledoc false
  # A source the test drives by hand. It reports each subscription and request
  # to the test process, and recognises inputs the test sends straight to the
  # estimator's process.
  @behaviour BB.Estimator.Source

  @impl BB.Estimator.Source
  def subscribe(robot, path, args) do
    recipient = :persistent_term.get({__MODULE__, robot})
    send(recipient, {__MODULE__, :subscribed, self(), path, args})
    {:ok, recipient}
  end

  @impl BB.Estimator.Source
  def input_event(_recipient, {__MODULE__, :input, input, monotonic_time, node}),
    do: {:input, input, monotonic_time, node}

  def input_event(_recipient, {__MODULE__, :no_input}), do: :no_input
  def input_event(_recipient, {__MODULE__, :spent}), do: :spent
  def input_event(_recipient, {__MODULE__, :ended, reason}), do: {:ended, reason}
  def input_event(_recipient, _message), do: nil

  @impl BB.Estimator.Source
  def request(recipient) do
    send(recipient, {__MODULE__, :requested, self()})
    :ok
  end
end

defmodule BB.Estimator.SourceTest.FakeSourceSensor do
  @moduledoc false
  use BB.Sensor

  @impl BB.Sensor
  def init(opts), do: {:ok, %{bb: Keyword.fetch!(opts, :bb)}}

  @impl BB.Sensor
  def estimator_source, do: {BB.Estimator.SourceTest.FakeSource, tag: :fake}
end

defmodule BB.Estimator.SourceTest do
  use ExUnit.Case, async: false

  alias BB.Estimator.SourceTest.{FakeSource, FakeSourceSensor}
  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message
  alias BB.Message.Sensor.Imu
  alias BB.Test.RecordingEstimator

  @sensor_path [:sensor, :base_link, :eye]

  defmodule RecordingRobot do
    @moduledoc false
    use BB

    topology do
      link :base_link do
        sensor :eye, FakeSourceSensor do
          estimator :recorder, RecordingEstimator do
            max_input_age(~u(100 millisecond))
          end
        end
      end
    end
  end

  defmodule GatedRobot do
    @moduledoc false
    use BB

    topology do
      link :base_link do
        sensor :eye, FakeSourceSensor do
          estimator :recorder, {RecordingEstimator, gated: true}
        end
      end
    end
  end

  defmodule EchoRobot do
    @moduledoc false
    use BB

    topology do
      link :base_link do
        sensor :eye, FakeSourceSensor do
          estimator :echo, EchoEstimator do
            max_input_age(~u(100 millisecond))
          end
        end
      end
    end
  end

  defmodule LostRobot do
    @moduledoc false
    use BB

    topology do
      link :base_link do
        sensor :eye, FakeSourceSensor do
          estimator :recorder, RecordingEstimator do
            lost_after(~u(1 millisecond))
          end
        end
      end
    end
  end

  setup do
    for robot <- [RecordingRobot, GatedRobot, EchoRobot, LostRobot] do
      :persistent_term.put({FakeSource, robot}, self())
      :persistent_term.put({RecordingEstimator, robot}, self())
    end

    on_exit(fn ->
      for robot <- [RecordingRobot, GatedRobot, EchoRobot, LostRobot] do
        :persistent_term.erase({FakeSource, robot})
        :persistent_term.erase({RecordingEstimator, robot})
      end
    end)
  end

  describe "a sensor that names a source" do
    test "its estimator subscribes through it, at the sensor's path, with its args" do
      start_supervised!(RecordingRobot)

      assert_receive {FakeSource, :subscribed, pid, @sensor_path, [tag: :fake]}
      assert pid == BB.Process.whereis(RecordingRobot, :recorder)
    end

    test "the estimator asks for an input once it has started" do
      start_supervised!(RecordingRobot)

      assert_receive {FakeSource, :subscribed, pid, _, _}
      assert_receive {FakeSource, :requested, ^pid}
    end

    test "handle_input receives whatever the source yields, not a message" do
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :input, :a_frame, now(), node()})

      assert_receive {:received, :input, :a_frame}
    end

    test "a message published at the sensor's path is not an input" do
      start_supervised!(RecordingRobot)
      pid = await_started()
      {:ok, _} = BB.subscribe(RecordingRobot, @sensor_path)

      BB.publish(RecordingRobot, @sensor_path, imu_message())
      assert_receive {:bb, @sensor_path, %Message{}}

      :sys.get_state(pid)
      refute_received {:received, :input, _}
    end

    test "the estimator asks again once it has dispatched an input" do
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :input, :a_frame, now(), node()})

      assert_receive {:received, :input, :a_frame}
      assert_receive {FakeSource, :requested, ^pid}
    end

    test "a message carrying no input is neither dispatched nor asked again" do
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :no_input})

      :sys.get_state(pid)
      refute_received {FakeSource, :requested, ^pid}
      refute_received {:received, :input, _}
    end

    test "a message that spent the request without an input is asked again" do
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :spent})

      assert_receive {FakeSource, :requested, ^pid}
      refute_received {:received, :input, _}
    end

    test "nothing is asked for while handle_input is held open" do
      start_supervised!(GatedRobot)
      pid = await_started()

      send(pid, {FakeSource, :input, :first, now(), node()})
      assert_receive {:received, :holding, :first}

      # The request and the holding report both come from the estimator's
      # process, so a request made before the hold would already be here.
      refute_received {FakeSource, :requested, ^pid}

      send(pid, :release)
      assert_receive {:received, :input, :first}
      assert_receive {FakeSource, :requested, ^pid}
    end

    test "an ended source stops the estimator, which subscribes afresh" do
      start_supervised!(RecordingRobot)
      pid = await_started()
      monitor = Process.monitor(pid)

      send(pid, {FakeSource, :ended, :camera_gone})

      assert_receive {:DOWN, ^monitor, :process, ^pid,
                      {:shutdown, {:source_ended, :parent, :camera_gone}}}

      assert_receive {FakeSource, :subscribed, new_pid, @sensor_path, _}
      assert new_pid != pid
    end
  end

  describe "health is measured from the source's timestamp" do
    test "an input older than max_input_age by the source's clock is dropped" do
      attach_telemetry(EchoRobot, [[:bb, :estimator, :dropped], [:bb, :estimator, :transition]])
      start_supervised!(EchoRobot)
      pid = await_started()
      {:ok, _} = BB.subscribe(EchoRobot, @sensor_path ++ [:echo])

      fresh_message = imu_message()
      send(pid, {FakeSource, :input, fresh_message, now() - 1_000_000_000, node()})

      assert_receive {:telemetry, :dropped, %{reason: :stale_input, source_input: :parent}}
      assert_receive {:telemetry, :transition, %{to: :degraded, reason: :stale_input}}
      :sys.get_state(pid)
      refute_received {:bb, _, %Message{}}
    end

    test "an input fresh by the source's clock is dispatched" do
      start_supervised!(EchoRobot)
      pid = await_started()
      {:ok, _} = BB.subscribe(EchoRobot, @sensor_path ++ [:echo])

      send(pid, {FakeSource, :input, imu_message(), now(), node()})

      assert_receive {:bb, _, %Message{payload: %Imu{}}}
    end

    test "an input produced on another node is refused" do
      attach_telemetry(RecordingRobot, [[:bb, :estimator, :dropped]])
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :input, :a_frame, now(), :other@nowhere})

      assert_receive {:telemetry, :dropped, %{reason: :cross_node}}
      :sys.get_state(pid)
      refute_received {:received, :input, _}
    end

    test "latency is measured from the source's timestamp" do
      attach_telemetry(EchoRobot, [[:bb, :estimator, :latency]])
      start_supervised!(EchoRobot)
      pid = await_started()

      age_ns = 50_000_000
      send(pid, {FakeSource, :input, imu_message(), now() - age_ns, node()})

      assert_receive {:telemetry, :latency, %{input_to_output: input_to_output}}
      assert input_to_output >= age_ns
    end

    test "an input recovers an estimator that was lost" do
      attach_telemetry(LostRobot, [[:bb, :estimator, :transition]])
      start_supervised!(LostRobot)
      pid = await_started()

      assert_receive {:telemetry, :transition, %{to: :lost}}

      send(pid, {FakeSource, :input, :a_frame, now(), node()})

      assert_receive {:received, :input, :a_frame}
      assert_receive {:telemetry, :transition, %{from: :lost, to: :degraded, reason: :recovered}}
    end
  end

  defp await_started do
    assert_receive {FakeSource, :subscribed, pid, _, _}
    assert_receive {FakeSource, :requested, ^pid}
    pid
  end

  defp attach_telemetry(robot, events) do
    handler_id = "source-test-#{:erlang.unique_integer([:positive])}"
    test_pid = self()

    :telemetry.attach_many(
      handler_id,
      events,
      fn
        event, measurements, %{robot: ^robot} = metadata, _ ->
          send(test_pid, {:telemetry, List.last(event), Map.merge(measurements, metadata)})

        _event, _measurements, _metadata, _ ->
          :ok
      end,
      nil
    )

    on_exit(fn -> :telemetry.detach(handler_id) end)
  end

  defp now, do: System.monotonic_time(:nanosecond)

  defp imu_message do
    {:ok, message} =
      Imu.new(:eye,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.zero()
      )

    message
  end
end
