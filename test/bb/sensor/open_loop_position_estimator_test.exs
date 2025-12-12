# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.OpenLoopPositionEstimatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Sensor.OpenLoopPositionEstimator

  import BB.Unit

  @joint_name :test_joint
  @sensor_name :test_feedback
  @actuator_name :test_servo

  defp begin_motion(initial, target, expected_arrival) do
    Message.new!(BeginMotion, @joint_name,
      initial_position: initial,
      target_position: target,
      expected_arrival: expected_arrival
    )
  end

  defp default_bb_context do
    %{robot: TestRobot, path: [@joint_name, @sensor_name]}
  end

  defp init_sensor(opts \\ []) do
    stub(BB.PubSub, :subscribe, fn _robot, _path, _opts -> :ok end)

    default_opts = [bb: default_bb_context(), actuator: @actuator_name]
    {:ok, state, _timeout} = OpenLoopPositionEstimator.init(Keyword.merge(default_opts, opts))

    state
  end

  describe "init/1" do
    test "subscribes to actuator topic" do
      test_pid = self()

      expect(BB.PubSub, :subscribe, fn robot, path, _opts ->
        send(test_pid, {:subscribed, robot, path})
        :ok
      end)

      opts = [bb: default_bb_context(), actuator: @actuator_name]
      {:ok, _state, _timeout} = OpenLoopPositionEstimator.init(opts)

      assert_receive {:subscribed, TestRobot, [:actuator, @joint_name, @actuator_name]}
    end

    test "returns max_silence timeout" do
      stub(BB.PubSub, :subscribe, fn _robot, _path, _opts -> :ok end)

      opts = [bb: default_bb_context(), actuator: @actuator_name, max_silence: ~u(2 second)]
      {:ok, _state, timeout} = OpenLoopPositionEstimator.init(opts)

      assert timeout == 2000
    end

    test "calculates correct publish interval from rate" do
      state = init_sensor(publish_rate: ~u(100 hertz))

      assert state.publish_interval_ms == 10
    end

    test "calculates correct max silence interval" do
      state = init_sensor(max_silence: ~u(10 second))

      assert state.max_silence_ms == 10_000
    end

    test "defaults to linear easing" do
      state = init_sensor()

      assert state.easing == :linear
    end

    test "accepts custom easing function" do
      state = init_sensor(easing: :ease_in_out_quad)

      assert state.easing == :ease_in_out_quad
    end
  end

  describe "begin_motion handling" do
    test "stores initial position, target and expected arrival" do
      state = init_sensor()
      expected_arrival = System.monotonic_time(:millisecond) + 500

      {:noreply, new_state, _timeout} =
        OpenLoopPositionEstimator.handle_info(begin_motion(0.0, 0.5, expected_arrival), state)

      assert new_state.initial_position == 0.0
      assert new_state.target_position == 0.5
      assert new_state.expected_arrival == expected_arrival
    end

    test "schedules tick when motion is in progress" do
      state = init_sensor()
      expected_arrival = System.monotonic_time(:millisecond) + 500

      {:noreply, new_state, _timeout} =
        OpenLoopPositionEstimator.handle_info(begin_motion(0.0, 0.5, expected_arrival), state)

      assert new_state.tick_ref != nil
      assert_receive :tick, 100
    end

    test "publishes immediately when motion already complete" do
      state = init_sensor()
      test_pid = self()
      arrival = System.monotonic_time(:millisecond) - 100

      expect(BB.PubSub, :publish, fn robot, path, message ->
        send(test_pid, {:published, robot, path, message})
        :ok
      end)

      {:noreply, new_state, _timeout} =
        OpenLoopPositionEstimator.handle_info(begin_motion(0.0, 0.5, arrival), state)

      assert_receive {:published, TestRobot, [:sensor, @joint_name, @sensor_name], message}
      assert message.payload.positions == [0.5]
      assert new_state.tick_ref == nil
    end

    test "cancels existing tick when new motion arrives" do
      state = init_sensor()
      arrival1 = System.monotonic_time(:millisecond) + 500

      {:noreply, state, _timeout} =
        OpenLoopPositionEstimator.handle_info(begin_motion(0.0, 0.5, arrival1), state)

      old_ref = state.tick_ref
      arrival2 = System.monotonic_time(:millisecond) + 500

      {:noreply, new_state, _timeout} =
        OpenLoopPositionEstimator.handle_info(begin_motion(0.5, 1.0, arrival2), state)

      assert new_state.tick_ref != old_ref
      assert new_state.initial_position == 0.5
      assert new_state.target_position == 1.0
    end
  end

  describe "tick behaviour" do
    test "ignores tick when tick_ref is nil (cancelled)" do
      state = init_sensor()
      state = %{state | tick_ref: nil, target_position: 0.5}

      reject(&BB.PubSub.publish/3)

      {:noreply, _state, _timeout} = OpenLoopPositionEstimator.handle_info(:tick, state)
    end

    test "publishes final position when motion complete" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)
      test_pid = self()

      state = %{
        state
        | target_position: 0.5,
          expected_arrival: now - 100,
          initial_position: 0.0,
          command_time: now - 200,
          tick_ref: make_ref()
      }

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, new_state, _timeout} = OpenLoopPositionEstimator.handle_info(:tick, state)

      assert_receive {:position, 0.5}
      assert new_state.tick_ref == nil
    end

    test "publishes interpolated position and schedules next tick during motion" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)
      test_pid = self()

      state = %{
        state
        | target_position: 1.0,
          expected_arrival: now + 1000,
          initial_position: 0.0,
          command_time: now - 100,
          tick_ref: make_ref()
      }

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, new_state, _timeout} = OpenLoopPositionEstimator.handle_info(:tick, state)

      assert_receive {:position, position}
      assert position > 0.0
      assert position < 1.0
      assert new_state.tick_ref != nil
      assert_receive :tick, 100
    end

    test "does not publish when position unchanged" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)

      state = %{
        state
        | target_position: 0.5,
          expected_arrival: now + 1000,
          initial_position: 0.5,
          command_time: now,
          tick_ref: make_ref(),
          last_published: 0.5
      }

      reject(&BB.PubSub.publish/3)

      {:noreply, new_state, _timeout} = OpenLoopPositionEstimator.handle_info(:tick, state)

      assert new_state.tick_ref != nil
    end
  end

  describe "timeout behaviour" do
    test "publishes current position on timeout when target is set" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)
      test_pid = self()

      state = %{
        state
        | target_position: 0.5,
          expected_arrival: now - 100,
          initial_position: 0.0,
          command_time: now - 200
      }

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, _state, timeout} = OpenLoopPositionEstimator.handle_info(:timeout, state)

      assert_receive {:position, 0.5}
      assert timeout == state.max_silence_ms
    end

    test "does nothing on timeout when no target set" do
      state = init_sensor()

      reject(&BB.PubSub.publish/3)

      {:noreply, _state, timeout} = OpenLoopPositionEstimator.handle_info(:timeout, state)

      assert timeout == state.max_silence_ms
    end
  end

  describe "position interpolation" do
    test "interpolates position during movement" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)
      test_pid = self()

      state = %{
        state
        | target_position: 1.0,
          expected_arrival: now + 1000,
          initial_position: 0.0,
          command_time: now,
          tick_ref: make_ref()
      }

      Process.sleep(100)

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, _state, _timeout} = OpenLoopPositionEstimator.handle_info(:tick, state)

      assert_receive {:position, position}
      assert position > 0.0
      assert position < 1.0
    end

    test "interpolates negative movement correctly" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)
      test_pid = self()

      state = %{
        state
        | target_position: -1.0,
          expected_arrival: now + 1000,
          initial_position: 0.0,
          command_time: now,
          tick_ref: make_ref()
      }

      Process.sleep(100)

      expect(BB.PubSub, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, _state, _timeout} = OpenLoopPositionEstimator.handle_info(:tick, state)

      assert_receive {:position, position}
      assert position < 0.0
      assert position > -1.0
    end
  end

  describe "easing functions" do
    test "ease_in_quad produces slower start than linear" do
      test_pid = self()

      expect(BB.PubSub, :publish, 2, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      now = System.monotonic_time(:millisecond)

      linear_state = init_sensor(easing: :linear)

      linear_state = %{
        linear_state
        | target_position: 1.0,
          expected_arrival: now + 800,
          initial_position: 0.0,
          command_time: now - 200,
          tick_ref: make_ref()
      }

      quad_state = init_sensor(easing: :ease_in_quad)

      quad_state = %{
        quad_state
        | target_position: 1.0,
          expected_arrival: now + 800,
          initial_position: 0.0,
          command_time: now - 200,
          tick_ref: make_ref()
      }

      {:noreply, _, _} = OpenLoopPositionEstimator.handle_info(:tick, linear_state)
      assert_receive {:position, linear_pos}

      {:noreply, _, _} = OpenLoopPositionEstimator.handle_info(:tick, quad_state)
      assert_receive {:position, quad_pos}

      assert quad_pos < linear_pos, "ease_in should be slower at start"
    end

    test "ease_out_quad produces faster start than linear" do
      test_pid = self()

      expect(BB.PubSub, :publish, 2, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      now = System.monotonic_time(:millisecond)

      linear_state = init_sensor(easing: :linear)

      linear_state = %{
        linear_state
        | target_position: 1.0,
          expected_arrival: now + 800,
          initial_position: 0.0,
          command_time: now - 200,
          tick_ref: make_ref()
      }

      quad_state = init_sensor(easing: :ease_out_quad)

      quad_state = %{
        quad_state
        | target_position: 1.0,
          expected_arrival: now + 800,
          initial_position: 0.0,
          command_time: now - 200,
          tick_ref: make_ref()
      }

      {:noreply, _, _} = OpenLoopPositionEstimator.handle_info(:tick, linear_state)
      assert_receive {:position, linear_pos}

      {:noreply, _, _} = OpenLoopPositionEstimator.handle_info(:tick, quad_state)
      assert_receive {:position, quad_pos}

      assert quad_pos > linear_pos, "ease_out should be faster at start"
    end

    test "all easing functions reach target at completion" do
      easings = [
        :linear,
        :ease_in_quad,
        :ease_out_quad,
        :ease_in_out_quad,
        :ease_in_cubic,
        :ease_out_cubic
      ]

      for easing <- easings do
        state = init_sensor(easing: easing)
        now = System.monotonic_time(:millisecond)

        state = %{
          state
          | target_position: 0.75,
            expected_arrival: now - 100,
            initial_position: 0.0,
            command_time: now - 1000,
            tick_ref: make_ref()
        }

        test_pid = self()

        expect(BB.PubSub, :publish, fn _robot, _path, message ->
          send(test_pid, {:position, hd(message.payload.positions)})
          :ok
        end)

        {:noreply, _, _} = OpenLoopPositionEstimator.handle_info(:tick, state)

        assert_receive {:position, position}
        assert position == 0.75, "#{easing} should reach target position"
      end
    end
  end
end
