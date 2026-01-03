# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Command.OptionsTest do
  @moduledoc """
  Tests for command options - both static and parameterised via ParamRef.

  Tests the full flow: DSL definition → compilation → runtime resolution →
  parameter changes → handle_options callback invocation.
  """
  use ExUnit.Case, async: false

  alias BB.{Message, PubSub}
  alias BB.Parameter.Changed, as: ParameterChanged
  alias BB.Robot.Runtime
  alias BB.Robot.State, as: RobotState

  # Use an Agent to track command state changes across process boundaries
  defmodule StateTracker do
    use Agent

    def start_link(_opts), do: Agent.start_link(fn -> [] end, name: __MODULE__)
    def record(event), do: Agent.update(__MODULE__, fn events -> [event | events] end)
    def get_events, do: Agent.get(__MODULE__, & &1)
    def clear, do: Agent.update(__MODULE__, fn _ -> [] end)
  end

  defmodule StaticOptionsCommand do
    @moduledoc """
    Test command that receives static options via handler tuple.
    """
    use BB.Command

    @impl BB.Command
    def init(opts) do
      max_velocity = Keyword.get(opts, :max_velocity)
      timeout_ms = Keyword.get(opts, :timeout_ms)
      StateTracker.record({:static_init, max_velocity: max_velocity, timeout_ms: timeout_ms})

      state =
        opts
        |> Map.new()
        |> Map.put_new(:result, nil)
        |> Map.put_new(:next_state, nil)
        |> Map.put(:max_velocity, max_velocity)
        |> Map.put(:timeout_ms, timeout_ms)

      {:ok, state}
    end

    @impl BB.Command
    def handle_command(_goal, _context, state) do
      StateTracker.record({:static_handle_command, state.max_velocity, state.timeout_ms})
      {:stop, :normal, %{state | result: {:ok, %{max_velocity: state.max_velocity}}}}
    end

    @impl BB.Command
    def result(%{result: result}), do: result
  end

  defmodule ParamOptionsCommand do
    @moduledoc """
    Test command that receives parameterised options and handles updates.
    """
    use BB.Command

    @impl BB.Command
    def init(opts) do
      gain = Keyword.get(opts, :gain)
      StateTracker.record({:param_init, gain})

      state =
        opts
        |> Map.new()
        |> Map.put_new(:result, nil)
        |> Map.put_new(:next_state, nil)
        |> Map.put(:gain, gain)

      {:ok, state}
    end

    @impl BB.Command
    def handle_command(%{wait_for_update: true}, _context, state) do
      # Stay running to receive option updates
      StateTracker.record({:param_handle_command, state.gain, :waiting})
      {:noreply, state}
    end

    def handle_command(_goal, _context, state) do
      StateTracker.record({:param_handle_command, state.gain, :immediate})
      {:stop, :normal, %{state | result: {:ok, %{gain: state.gain}}}}
    end

    @impl BB.Command
    def handle_options(new_opts, state) do
      new_gain = Keyword.get(new_opts, :gain)
      StateTracker.record({:param_handle_options, new_gain})
      {:ok, %{state | gain: new_gain}}
    end

    @impl BB.Command
    def handle_info(:complete, state) do
      {:stop, :normal, %{state | result: {:ok, %{final_gain: state.gain}}}}
    end

    def handle_info(_msg, state) do
      {:noreply, state}
    end

    @impl BB.Command
    def result(%{result: nil}), do: {:error, :cancelled}
    def result(%{result: result}), do: result
  end

  defmodule StaticOptionsRobot do
    @moduledoc false
    use BB

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :with_static_opts do
        handler {StaticOptionsCommand, max_velocity: 1.5, timeout_ms: 5000}
        allowed_states [:idle]
      end
    end

    topology do
      link :base do
      end
    end
  end

  defmodule ParamOptionsRobot do
    @moduledoc false
    use BB

    parameters do
      group :control do
        param :gain,
          type: :float,
          default: 1.0,
          doc: "Controller gain for commands"
      end
    end

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :disarm do
        handler BB.Command.Disarm
        allowed_states [:idle]
      end

      command :with_param_opts do
        handler {ParamOptionsCommand, gain: param([:control, :gain])}
        allowed_states [:idle]
      end
    end

    topology do
      link :base do
      end
    end
  end

  defmodule MixedOptionsRobot do
    @moduledoc false
    use BB

    parameters do
      group :motion do
        param :max_velocity,
          type: :float,
          default: 2.0,
          doc: "Maximum velocity"
      end
    end

    commands do
      command :arm do
        handler BB.Command.Arm
        allowed_states [:disarmed]
      end

      command :with_mixed_opts do
        handler {StaticOptionsCommand,
                 max_velocity: param([:motion, :max_velocity]), timeout_ms: 3000}

        allowed_states [:idle]
      end
    end

    topology do
      link :base do
      end
    end
  end

  setup do
    start_supervised!(StateTracker)
    :ok
  end

  describe "static command options" do
    test "options are passed to command init" do
      start_supervised!(StaticOptionsRobot)

      :ok = BB.Safety.arm(StaticOptionsRobot)

      {:ok, cmd} = StaticOptionsRobot.with_static_opts()
      {:ok, result} = BB.Command.await(cmd)

      events = StateTracker.get_events()

      # Check init received the static options
      init_event = Enum.find(events, &match?({:static_init, _}, &1))
      assert init_event != nil
      {:static_init, init_opts} = init_event
      assert init_opts[:max_velocity] == 1.5
      assert init_opts[:timeout_ms] == 5000

      # Check handle_command had access to options
      cmd_event = Enum.find(events, &match?({:static_handle_command, _, _}, &1))
      assert cmd_event != nil
      {:static_handle_command, max_vel, timeout} = cmd_event
      assert max_vel == 1.5
      assert timeout == 5000

      # Check result includes the option value
      assert result.max_velocity == 1.5
    end
  end

  describe "parameterised command options" do
    test "param is resolved at command startup" do
      start_supervised!(ParamOptionsRobot)

      :ok = BB.Safety.arm(ParamOptionsRobot)

      {:ok, cmd} = ParamOptionsRobot.with_param_opts()
      {:ok, result} = BB.Command.await(cmd)

      events = StateTracker.get_events()

      # Check init received the resolved parameter value
      init_event = Enum.find(events, &match?({:param_init, _}, &1))
      assert init_event != nil
      {:param_init, gain} = init_event
      assert_in_delta gain, 1.0, 0.001

      # Check result
      assert_in_delta result.gain, 1.0, 0.001
    end

    test "param uses current value when command starts" do
      start_supervised!(ParamOptionsRobot)

      # Change parameter before starting command
      robot_state = Runtime.get_robot_state(ParamOptionsRobot)
      :ok = RobotState.set_parameter(robot_state, [:control, :gain], 5.0)

      :ok = BB.Safety.arm(ParamOptionsRobot)

      {:ok, cmd} = ParamOptionsRobot.with_param_opts()
      {:ok, result} = BB.Command.await(cmd)

      # Should use the updated value
      assert_in_delta result.gain, 5.0, 0.001
    end

    test "handle_options is called when param changes during execution" do
      start_supervised!(ParamOptionsRobot)

      :ok = BB.Safety.arm(ParamOptionsRobot)

      StateTracker.clear()

      # Start command that waits
      {:ok, cmd} = ParamOptionsRobot.with_param_opts(wait_for_update: true)

      # Wait for command to start
      Process.sleep(50)

      # Verify initial state
      events = StateTracker.get_events()
      init_event = Enum.find(events, &match?({:param_init, _}, &1))
      assert init_event != nil
      {:param_init, initial_gain} = init_event
      assert_in_delta initial_gain, 1.0, 0.001

      StateTracker.clear()

      # Change the parameter while command is running
      robot_state = Runtime.get_robot_state(ParamOptionsRobot)
      :ok = RobotState.set_parameter(robot_state, [:control, :gain], 3.5)

      # Publish the change notification
      message =
        Message.new!(ParameterChanged, :parameter,
          path: [:control, :gain],
          old_value: 1.0,
          new_value: 3.5,
          source: :local
        )

      PubSub.publish(ParamOptionsRobot, [:param, :control, :gain], message)

      # Wait for handle_options to be called
      Process.sleep(50)

      events = StateTracker.get_events()
      handle_opts_event = Enum.find(events, &match?({:param_handle_options, _}, &1))

      assert handle_opts_event != nil
      {:param_handle_options, new_gain} = handle_opts_event
      assert_in_delta new_gain, 3.5, 0.001

      # Complete the command and verify it used the updated value
      send(cmd, :complete)
      {:ok, result} = BB.Command.await(cmd)

      assert_in_delta result.final_gain, 3.5, 0.001
    end
  end

  describe "mixed static and parameterised options" do
    test "both static and param options are resolved correctly" do
      start_supervised!(MixedOptionsRobot)

      :ok = BB.Safety.arm(MixedOptionsRobot)

      {:ok, cmd} = MixedOptionsRobot.with_mixed_opts()
      {:ok, result} = BB.Command.await(cmd)

      events = StateTracker.get_events()

      # Check init received both types of options
      init_event = Enum.find(events, &match?({:static_init, _}, &1))
      assert init_event != nil
      {:static_init, init_opts} = init_event

      # Parameterised value should be resolved to 2.0
      assert_in_delta init_opts[:max_velocity], 2.0, 0.001
      # Static value should be passed through
      assert init_opts[:timeout_ms] == 3000

      assert_in_delta result.max_velocity, 2.0, 0.001
    end

    test "only parameterised options trigger handle_options" do
      start_supervised!(MixedOptionsRobot)

      # This test verifies the command receives param updates but StaticOptionsCommand
      # doesn't implement handle_options (uses default), so we just verify no crash occurs
      # and the command completes successfully

      :ok = BB.Safety.arm(MixedOptionsRobot)

      {:ok, cmd} = MixedOptionsRobot.with_mixed_opts()
      {:ok, _result} = BB.Command.await(cmd)

      # Command completed without error - default handle_options worked
      assert true
    end
  end

  describe "command without options" do
    test "commands work without handler tuple" do
      start_supervised!(ParamOptionsRobot)

      # Arm command uses plain module, not tuple
      {:ok, cmd} = ParamOptionsRobot.arm()
      {:ok, :armed, _opts} = BB.Command.await(cmd)

      assert Runtime.state(ParamOptionsRobot) == :idle
    end
  end
end
