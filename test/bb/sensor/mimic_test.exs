# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.MimicTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Sensor.Mimic

  @source_joint :left_finger
  @mimic_joint :right_finger
  @sensor_name :mimic

  defp joint_state_message(names, positions) do
    Message.new!(JointState, :gripper,
      names: names,
      positions: positions
    )
  end

  defp default_bb_context do
    %{robot: TestRobot, path: [:gripper_link, @mimic_joint, @sensor_name]}
  end

  defp init_sensor(opts \\ []) do
    stub(BB.PubSub, :subscribe, fn _robot, _path, _opts -> :ok end)

    default_opts = [bb: default_bb_context(), source: @source_joint]
    {:ok, state} = Mimic.init(Keyword.merge(default_opts, opts))

    state
  end

  describe "init/1" do
    test "subscribes to source joint's sensor topic" do
      test_pid = self()

      expect(BB.PubSub, :subscribe, fn robot, path, opts ->
        send(test_pid, {:subscribed, robot, path, opts})
        :ok
      end)

      opts = [bb: default_bb_context(), source: @source_joint]
      {:ok, _state} = Mimic.init(opts)

      assert_receive {:subscribed, TestRobot, [:sensor, :gripper_link, @source_joint], opts}
      assert opts[:message_types] == [JointState]
    end

    test "defaults to multiplier of 1.0" do
      state = init_sensor()

      assert state.multiplier == 1.0
    end

    test "defaults to offset of 0.0" do
      state = init_sensor()

      assert state.offset == 0.0
    end

    test "accepts custom multiplier" do
      state = init_sensor(multiplier: -1.0)

      assert state.multiplier == -1.0
    end

    test "accepts custom offset" do
      state = init_sensor(offset: 0.01)

      assert state.offset == 0.01
    end

    test "stores source joint name" do
      state = init_sensor()

      assert state.source == @source_joint
    end

    test "stores mimic joint name" do
      state = init_sensor()

      assert state.joint_name == @mimic_joint
    end
  end

  describe "handle_info/2 with JointState" do
    test "transforms position with default multiplier and offset" do
      state = init_sensor()
      test_pid = self()

      expect(BB.PubSub, :publish, fn robot, path, message ->
        send(test_pid, {:published, robot, path, message})
        :ok
      end)

      source_message = joint_state_message([@source_joint], [0.025])

      {:noreply, _state} =
        Mimic.handle_info({:bb, [:sensor, :gripper_link, @source_joint], source_message}, state)

      assert_receive {:published, TestRobot, [:sensor, :gripper_link, @mimic_joint, @sensor_name],
                      message}

      assert message.payload.names == [@mimic_joint]
      assert message.payload.positions == [0.025]
    end

    test "applies multiplier to position" do
      state = init_sensor(multiplier: -1.0)
      test_pid = self()

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, message.payload.positions})
        :ok
      end)

      source_message = joint_state_message([@source_joint], [0.025])

      {:noreply, _state} =
        Mimic.handle_info({:bb, [:sensor, :gripper_link, @source_joint], source_message}, state)

      assert_receive {:position, [-0.025]}
    end

    test "applies offset to position" do
      state = init_sensor(offset: 0.01)
      test_pid = self()

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, message.payload.positions})
        :ok
      end)

      source_message = joint_state_message([@source_joint], [0.025])

      {:noreply, _state} =
        Mimic.handle_info({:bb, [:sensor, :gripper_link, @source_joint], source_message}, state)

      assert_receive {:position, [0.035]}
    end

    test "applies both multiplier and offset correctly" do
      state = init_sensor(multiplier: 2.0, offset: 0.01)
      test_pid = self()

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, message.payload.positions})
        :ok
      end)

      source_message = joint_state_message([@source_joint], [0.025])

      {:noreply, _state} =
        Mimic.handle_info({:bb, [:sensor, :gripper_link, @source_joint], source_message}, state)

      assert_receive {:position, [position]}
      assert_in_delta position, 0.06, 0.0001
    end

    test "replaces joint name with mimic joint name" do
      state = init_sensor()
      test_pid = self()

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:names, message.payload.names})
        :ok
      end)

      source_message = joint_state_message([@source_joint], [0.025])

      {:noreply, _state} =
        Mimic.handle_info({:bb, [:sensor, :gripper_link, @source_joint], source_message}, state)

      assert_receive {:names, [@mimic_joint]}
    end
  end

  describe "handle_info/2 with non-JointState messages" do
    test "forwards message without transformation" do
      state = init_sensor(message_types: [JointState, SomeOtherType])
      test_pid = self()

      other_message = %Message{
        timestamp: System.monotonic_time(:nanosecond),
        frame_id: :gripper,
        payload: %{__struct__: SomeOtherType, value: 42}
      }

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:forwarded, message})
        :ok
      end)

      {:noreply, _state} =
        Mimic.handle_info({:bb, [:sensor, :gripper_link, @source_joint], other_message}, state)

      assert_receive {:forwarded, ^other_message}
    end
  end
end
