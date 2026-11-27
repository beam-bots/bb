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
    sensor = BB.Process.whereis(robot, List.last(path))
    send(recipient, {__MODULE__, :sensor_at_subscribe, self(), sensor})
    {:ok, recipient}
  end

  @impl BB.Estimator.Source
  def input_event(_recipient, {__MODULE__, :input, input, monotonic_time, node}),
    do: {:input, input, monotonic_time, node}

  def input_event(_recipient, {__MODULE__, :no_input}), do: :no_input
  def input_event(_recipient, {__MODULE__, :spent}), do: :spent
  def input_event(_recipient, {__MODULE__, :failed, reason}), do: {:failed, reason}
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

defmodule BB.Estimator.SourceTest.NoteLost do
  @moduledoc false
  # An `on_lost` command that tells the test process it ran, and with what.
  use BB.Command

  @impl BB.Command
  def handle_command(goal, context, state) do
    send(:persistent_term.get({__MODULE__, context.robot_module}), {__MODULE__, goal})
    {:stop, :normal, %{state | result: {:ok, :noted}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end

defmodule BB.Estimator.SourceTest do
  use ExUnit.Case, async: false

  alias BB.Estimator.SourceTest.{FakeSource, FakeSourceSensor, NoteLost}
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

  defmodule GivingUpRobot do
    @moduledoc false
    use BB

    settings do
      sensor_max_restarts(0)
    end

    topology do
      link :base_link do
        sensor :eye, FakeSourceSensor do
          estimator :recorder, RecordingEstimator
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

  defmodule OnLostRobot do
    @moduledoc false
    use BB

    commands do
      command :note_lost do
        handler NoteLost
        allowed_states [:disarmed, :idle]
      end
    end

    topology do
      link :base_link do
        sensor :eye, FakeSourceSensor do
          estimator :recorder, RecordingEstimator do
            on_lost(:note_lost)
          end
        end
      end
    end
  end

  defmodule PubSubRobot do
    @moduledoc false
    use BB

    topology do
      link :base_link do
        sensor :imu, MySensor do
          estimator :recorder, RecordingEstimator
        end
      end
    end
  end

  @robots [
    RecordingRobot,
    GatedRobot,
    EchoRobot,
    LostRobot,
    OnLostRobot,
    PubSubRobot,
    GivingUpRobot
  ]

  setup do
    for robot <- @robots do
      :persistent_term.put({FakeSource, robot}, self())
      :persistent_term.put({RecordingEstimator, robot}, self())
      :persistent_term.put({NoteLost, robot}, self())
    end

    on_exit(fn ->
      for robot <- @robots do
        :persistent_term.erase({FakeSource, robot})
        :persistent_term.erase({RecordingEstimator, robot})
        :persistent_term.erase({NoteLost, robot})
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

    test "an ended source leaves the estimator lost, running, and no longer asking" do
      attach_telemetry(RecordingRobot, [[:bb, :estimator, :transition]])
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :ended, :camera_gone})

      assert_receive {:telemetry, :transition,
                      %{from: :healthy, to: :lost, reason: :source_ended}}

      :sys.get_state(pid)
      assert Process.alive?(pid)
      assert BB.Process.whereis(RecordingRobot, :recorder) == pid
      refute_received {FakeSource, :requested, ^pid}
      refute_received {FakeSource, :subscribed, _, _, _}
    end

    test "a failed source makes the estimator lost, and it asks again" do
      attach_telemetry(OnLostRobot, [[:bb, :estimator, :transition]])
      start_supervised!(OnLostRobot)
      pid = await_started()

      send(pid, {FakeSource, :failed, :capture_broke})

      assert_receive {:telemetry, :transition,
                      %{from: :healthy, to: :lost, reason: :source_failed}}

      assert_receive {FakeSource, :requested, ^pid}
      assert_receive {NoteLost, %{estimator: :recorder, reason: :source_failed}}
      assert BB.Process.whereis(OnLostRobot, :recorder) == pid
    end

    test "a failed source recovers on its next input" do
      attach_telemetry(RecordingRobot, [[:bb, :estimator, :transition]])
      start_supervised!(RecordingRobot)
      pid = await_started()

      send(pid, {FakeSource, :failed, :capture_broke})
      assert_receive {:telemetry, :transition, %{to: :lost, reason: :source_failed}}
      assert_receive {FakeSource, :requested, ^pid}

      send(pid, {FakeSource, :input, :a_frame, now(), node()})

      assert_receive {:received, :input, :a_frame}
      assert_receive {:telemetry, :transition, %{from: :lost, to: :degraded, reason: :recovered}}
    end
  end

  describe "a sensor-nested estimator restarts with its sensor" do
    test "a sensor crash restarts its estimator after it, which subscribes afresh" do
      start_supervised!(RecordingRobot)
      estimator = await_started()
      estimator_monitor = Process.monitor(estimator)

      sensor = BB.Process.whereis(RecordingRobot, :eye)
      Process.exit(sensor, :kill)

      assert_receive {:DOWN, ^estimator_monitor, :process, ^estimator, :shutdown}
      new_estimator = await_started()
      assert new_estimator != estimator

      new_sensor = BB.Process.whereis(RecordingRobot, :eye)
      assert is_pid(new_sensor) and new_sensor != sensor
    end

    test "a PubSub estimator restarts with its sensor too" do
      start_supervised!(PubSubRobot)
      estimator = BB.Process.whereis(PubSubRobot, :recorder)
      estimator_monitor = Process.monitor(estimator)
      group = sensor_group(PubSubRobot, :base_link, :imu)

      PubSubRobot
      |> BB.Process.whereis(:imu)
      |> Process.exit(:kill)

      assert_receive {:DOWN, ^estimator_monitor, :process, ^estimator, :shutdown}

      # The supervisor restarts the sensor and then the estimator while
      # handling the one exit, so a call to it lands after both have started.
      :sys.get_state(group)
      new_estimator = BB.Process.whereis(PubSubRobot, :recorder)
      assert is_pid(new_estimator) and new_estimator != estimator

      BB.publish(PubSubRobot, [:sensor, :base_link, :imu], imu_message())
      assert_receive {:received, :input, %Message{payload: %Imu{}}}
    end

    # The sensor's supervisor allows the default `sensor_max_restarts` of three. Were
    # the estimator's restart counted alongside the sensor's, the second crash
    # would exhaust it and take the supervisor down.
    test "a sensor crash and the estimator restart it causes count as one restart" do
      start_supervised!(RecordingRobot)
      await_started()
      group = sensor_group(RecordingRobot, :base_link, :eye)
      group_monitor = Process.monitor(group)

      for _ <- 1..3 do
        RecordingRobot
        |> BB.Process.whereis(:eye)
        |> Process.exit(:kill)

        await_started()
      end

      assert sensor_group(RecordingRobot, :base_link, :eye) == group
      refute_received {:DOWN, ^group_monitor, _, _, _}
    end

    test "a sensor that exhausts sensor_max_restarts escalates to its link" do
      start_supervised!(GivingUpRobot)
      await_started()
      link = GivingUpRobot |> BB.Process.whereis(BB.TopologySupervisor) |> child_pid(:base_link)
      group = sensor_group(GivingUpRobot, :base_link, :eye)
      group_monitor = Process.monitor(group)

      GivingUpRobot
      |> BB.Process.whereis(:eye)
      |> Process.exit(:kill)

      assert_receive {:DOWN, ^group_monitor, :process, ^group, :shutdown}

      # The link supervisor restarts the group while handling its exit, so a
      # call to it lands after the replacement has started.
      :sys.get_state(link)
      new_group = sensor_group(GivingUpRobot, :base_link, :eye)
      assert new_group != group
      await_started()
    end

    test "the restarted estimator subscribes once the new sensor is running" do
      start_supervised!(RecordingRobot)
      estimator = await_started()
      assert_receive {FakeSource, :sensor_at_subscribe, ^estimator, sensor}

      Process.exit(sensor, :kill)

      new_estimator = await_started()
      assert_receive {FakeSource, :sensor_at_subscribe, ^new_estimator, new_sensor}
      assert is_pid(new_sensor)
      assert new_sensor != sensor
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

  defp sensor_group(robot, link, sensor) do
    robot
    |> BB.Process.whereis(BB.TopologySupervisor)
    |> child_pid(link)
    |> child_pid(sensor)
  end

  defp child_pid(supervisor, id) do
    {^id, pid, :supervisor, _} =
      supervisor
      |> Supervisor.which_children()
      |> List.keyfind(id, 0)

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
