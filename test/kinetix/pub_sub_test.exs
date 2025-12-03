defmodule Kinetix.PubSubTest do
  use ExUnit.Case, async: false

  alias Kinetix.Message.{Geometry.Pose, Quaternion, Sensor.Imu, Vec3}
  alias Kinetix.PubSub

  defmodule TestRobot do
    @moduledoc false
    use Kinetix

    robot do
      link :base_link do
        joint :shoulder do
          type :revolute

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(100 degree_per_second))
          end

          link :arm do
          end
        end
      end
    end
  end

  describe "ancestor_paths/1" do
    test "generates all ancestor paths including self and root" do
      paths = PubSub.ancestor_paths([:sensor, :base_link, :joint1, :imu])

      assert paths == [
               [:sensor, :base_link, :joint1, :imu],
               [:sensor, :base_link, :joint1],
               [:sensor, :base_link],
               [:sensor],
               []
             ]
    end

    test "handles empty path" do
      assert PubSub.ancestor_paths([]) == [[]]
    end

    test "handles single-element path" do
      assert PubSub.ancestor_paths([:sensor]) == [[:sensor], []]
    end
  end

  describe "registry_name/1" do
    test "returns pubsub registry name for robot module" do
      assert PubSub.registry_name(TestRobot) == TestRobot.PubSub
    end
  end

  describe "subscribe/3 and publish/3" do
    test "subscriber receives message at exact path" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link, :shoulder, :imu]
      {:ok, _} = PubSub.subscribe(TestRobot, path)

      {:ok, message} =
        Imu.new(:imu_link,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.new(0, 0, 9.81)
        )

      PubSub.publish(TestRobot, path, message)

      assert_receive {:kinetix, ^path, ^message}
    end

    test "subscriber at parent path receives messages from children" do
      start_supervised!(TestRobot)
      parent_path = [:sensor, :base_link, :shoulder]
      child_path = [:sensor, :base_link, :shoulder, :imu]

      {:ok, _} = PubSub.subscribe(TestRobot, parent_path)

      {:ok, message} =
        Imu.new(:imu_link,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.new(0, 0, 9.81)
        )

      PubSub.publish(TestRobot, child_path, message)

      assert_receive {:kinetix, ^child_path, ^message}
    end

    test "subscriber to all sensors receives sensor messages" do
      start_supervised!(TestRobot)
      {:ok, _} = PubSub.subscribe(TestRobot, [:sensor])

      path = [:sensor, :base_link, :shoulder, :encoder]

      {:ok, message} =
        Imu.new(:encoder,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      PubSub.publish(TestRobot, path, message)

      assert_receive {:kinetix, ^path, ^message}
    end

    test "subscriber to empty path receives all messages" do
      start_supervised!(TestRobot)
      {:ok, _} = PubSub.subscribe(TestRobot, [])

      sensor_path = [:sensor, :base_link, :imu]
      actuator_path = [:actuator, :base_link, :shoulder, :motor]

      {:ok, sensor_msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      {:ok, actuator_msg} = Pose.new(:motor, Vec3.zero(), Quaternion.identity())

      PubSub.publish(TestRobot, sensor_path, sensor_msg)
      PubSub.publish(TestRobot, actuator_path, actuator_msg)

      assert_receive {:kinetix, ^sensor_path, ^sensor_msg}
      assert_receive {:kinetix, ^actuator_path, ^actuator_msg}
    end

    test "multiple subscribers receive the same message" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link, :imu]

      test_pid = self()

      spawn_link(fn ->
        {:ok, _} = PubSub.subscribe(TestRobot, path)
        send(test_pid, :subscriber1_ready)

        receive do
          {:kinetix, ^path, msg} -> send(test_pid, {:subscriber1, msg})
        end
      end)

      spawn_link(fn ->
        {:ok, _} = PubSub.subscribe(TestRobot, path)
        send(test_pid, :subscriber2_ready)

        receive do
          {:kinetix, ^path, msg} -> send(test_pid, {:subscriber2, msg})
        end
      end)

      assert_receive :subscriber1_ready
      assert_receive :subscriber2_ready

      {:ok, message} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      PubSub.publish(TestRobot, path, message)

      assert_receive {:subscriber1, ^message}
      assert_receive {:subscriber2, ^message}
    end
  end

  describe "message type filtering" do
    test "subscriber with message_types filter only receives matching types" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link]
      {:ok, _} = PubSub.subscribe(TestRobot, path, message_types: [Imu])

      imu_path = [:sensor, :base_link, :imu]
      pose_path = [:sensor, :base_link, :camera]

      {:ok, imu_msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      {:ok, pose_msg} = Pose.new(:camera, Vec3.zero(), Quaternion.identity())

      PubSub.publish(TestRobot, imu_path, imu_msg)
      PubSub.publish(TestRobot, pose_path, pose_msg)

      assert_receive {:kinetix, ^imu_path, ^imu_msg}
      refute_receive {:kinetix, ^pose_path, _}, 50
    end

    test "subscriber without message_types filter receives all types" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link]
      {:ok, _} = PubSub.subscribe(TestRobot, path)

      imu_path = [:sensor, :base_link, :imu]
      pose_path = [:sensor, :base_link, :camera]

      {:ok, imu_msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      {:ok, pose_msg} = Pose.new(:camera, Vec3.zero(), Quaternion.identity())

      PubSub.publish(TestRobot, imu_path, imu_msg)
      PubSub.publish(TestRobot, pose_path, pose_msg)

      assert_receive {:kinetix, ^imu_path, ^imu_msg}
      assert_receive {:kinetix, ^pose_path, ^pose_msg}
    end

    test "subscriber with empty message_types receives all types" do
      start_supervised!(TestRobot)
      path = [:sensor]
      {:ok, _} = PubSub.subscribe(TestRobot, path, message_types: [])

      imu_path = [:sensor, :base_link, :imu]

      {:ok, imu_msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      PubSub.publish(TestRobot, imu_path, imu_msg)

      assert_receive {:kinetix, ^imu_path, ^imu_msg}
    end

    test "subscriber with multiple message_types receives any matching type" do
      start_supervised!(TestRobot)
      path = [:sensor]
      {:ok, _} = PubSub.subscribe(TestRobot, path, message_types: [Imu, Pose])

      imu_path = [:sensor, :base_link, :imu]
      pose_path = [:sensor, :base_link, :camera]

      {:ok, imu_msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      {:ok, pose_msg} = Pose.new(:camera, Vec3.zero(), Quaternion.identity())

      PubSub.publish(TestRobot, imu_path, imu_msg)
      PubSub.publish(TestRobot, pose_path, pose_msg)

      assert_receive {:kinetix, ^imu_path, ^imu_msg}
      assert_receive {:kinetix, ^pose_path, ^pose_msg}
    end
  end

  describe "unsubscribe/2" do
    test "unsubscribed process no longer receives messages" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link, :imu]
      {:ok, _} = PubSub.subscribe(TestRobot, path)

      {:ok, message1} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.zero()
        )

      PubSub.publish(TestRobot, path, message1)
      assert_receive {:kinetix, ^path, ^message1}

      :ok = PubSub.unsubscribe(TestRobot, path)

      {:ok, message2} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.new(1, 0, 0),
          linear_acceleration: Vec3.zero()
        )

      PubSub.publish(TestRobot, path, message2)
      refute_receive {:kinetix, ^path, _}, 50
    end
  end

  describe "subscribers/2" do
    test "returns list of subscribers for a path" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link]
      {:ok, _} = PubSub.subscribe(TestRobot, path, message_types: [Imu])

      subscribers = PubSub.subscribers(TestRobot, path)

      assert [{pid, [Imu]}] = subscribers
      assert pid == self()
    end

    test "returns empty list when no subscribers" do
      start_supervised!(TestRobot)
      assert PubSub.subscribers(TestRobot, [:nonexistent, :path]) == []
    end
  end

  describe "process cleanup" do
    test "dead process is automatically unsubscribed" do
      start_supervised!(TestRobot)
      path = [:sensor, :base_link, :imu]

      pid =
        spawn(fn ->
          {:ok, _} = PubSub.subscribe(TestRobot, path)

          receive do
            :done -> :ok
          end
        end)

      Process.sleep(10)
      assert [{^pid, []}] = PubSub.subscribers(TestRobot, path)

      Process.exit(pid, :kill)
      Process.sleep(10)

      assert PubSub.subscribers(TestRobot, path) == []
    end
  end
end
