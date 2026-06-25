# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Safety.ControllerSafetyStateTest do
  @moduledoc """
  Tests for the `handle_safety_state_change/2` hook on `BB.Controller`.

  Unlike a `BB.Command` (short-lived, stops on disarm), a controller is
  long-lived: the default keeps it running across arm/disarm so it can gate its
  own output (command-silence) rather than be killed.

  These tests run synchronously because they drive the global safety state
  machine via `BB.Safety.arm/1` and `BB.Safety.disarm/1`.
  """
  use ExUnit.Case, async: false

  alias BB.Process, as: BBProcess

  # A controller that uses the DEFAULT handle_safety_state_change/2 (i.e. it does
  # not override it). The default must return {:continue, state} so the
  # controller stays alive through disarm.
  defmodule DefaultController do
    @moduledoc false
    use BB.Controller, options_schema: []

    @impl BB.Controller
    def init(opts) do
      {:ok, %{bb: opts[:bb]}}
    end

    @impl BB.Controller
    def handle_call(:get_state, _from, state) do
      {:reply, state, state}
    end
  end

  # The registered name the OverridingController notifies on a safety change.
  @notify_name :bb_controller_safety_state_test_notify

  # A controller that OVERRIDES handle_safety_state_change/2 to record the
  # callback firing (by sending to a registered test process) and to flip an
  # `armed?` flag in its state — the disarm-safe pattern of gating its own
  # output.
  defmodule OverridingController do
    @moduledoc false
    use BB.Controller, options_schema: []

    @notify_name :bb_controller_safety_state_test_notify

    @impl BB.Controller
    def init(opts) do
      {:ok, %{bb: opts[:bb], armed?: true, last_safety_state: nil}}
    end

    @impl BB.Controller
    def handle_safety_state_change(new_state, state) do
      case Process.whereis(@notify_name) do
        nil -> :ok
        pid -> send(pid, {:safety_state_change, new_state})
      end

      {:continue, %{state | armed?: false, last_safety_state: new_state}}
    end

    @impl BB.Controller
    def handle_call(:get_state, _from, state) do
      {:reply, state, state}
    end
  end

  # A controller that OVERRIDES the hook to STOP itself on disarm, proving the
  # {:stop, reason, state} return path is honoured by the server.
  defmodule StoppingController do
    @moduledoc false
    use BB.Controller, options_schema: []

    @impl BB.Controller
    def init(opts) do
      {:ok, %{bb: opts[:bb]}}
    end

    @impl BB.Controller
    def handle_safety_state_change(_new_state, state) do
      {:stop, :disarmed, state}
    end
  end

  defmodule DefaultControllerRobot do
    @moduledoc false
    use BB

    controllers do
      controller(:driver, BB.Safety.ControllerSafetyStateTest.DefaultController)
    end

    topology do
      link :base_link do
      end
    end
  end

  defmodule OverridingControllerRobot do
    @moduledoc false
    use BB

    controllers do
      controller(:driver, BB.Safety.ControllerSafetyStateTest.OverridingController)
    end

    topology do
      link :base_link do
      end
    end
  end

  defmodule StoppingControllerRobot do
    @moduledoc false
    use BB

    controllers do
      controller(:driver, BB.Safety.ControllerSafetyStateTest.StoppingController)
    end

    topology do
      link :base_link do
      end
    end
  end

  describe "default handle_safety_state_change/2" do
    test "controller stays alive across disarm (default returns {:continue})" do
      start_supervised!(DefaultControllerRobot)

      controller_pid = BBProcess.whereis(DefaultControllerRobot, :driver)
      assert is_pid(controller_pid)
      assert Process.alive?(controller_pid)

      :ok = BB.Safety.arm(DefaultControllerRobot)
      :ok = BB.Safety.disarm(DefaultControllerRobot)

      # Give the async safety-state message time to be delivered and handled.
      Process.sleep(50)

      # Same pid, still alive: the default did not stop the controller.
      assert BBProcess.whereis(DefaultControllerRobot, :driver) == controller_pid
      assert Process.alive?(controller_pid)
      assert match?(%{bb: _}, GenServer.call(controller_pid, :get_state))
    end
  end

  describe "overridden handle_safety_state_change/2" do
    test "override is invoked with the new safety state and can update state" do
      # The overriding controller notifies a process registered under a known
      # name; register this test process so we observe the callback firing.
      Process.register(self(), @notify_name)
      on_exit(fn -> safe_unregister(@notify_name) end)

      start_supervised!(OverridingControllerRobot)

      controller_pid = BBProcess.whereis(OverridingControllerRobot, :driver)
      assert is_pid(controller_pid)

      :ok = BB.Safety.arm(OverridingControllerRobot)
      :ok = BB.Safety.disarm(OverridingControllerRobot)

      # Disarm walks armed -> disarming -> disarmed, so the hook fires for the
      # disarming transition first.
      assert_receive {:safety_state_change, :disarming}, 1_000

      # Controller is still alive (override returned {:continue}) and recorded
      # the disarm by flipping its armed? flag.
      assert Process.alive?(controller_pid)
      state = GenServer.call(controller_pid, :get_state)
      assert state.armed? == false
      assert state.last_safety_state in [:disarming, :disarmed]
    end

    test "override returning {:stop, reason, state} stops the controller" do
      Process.flag(:trap_exit, true)

      start_supervised!(StoppingControllerRobot)

      controller_pid = BBProcess.whereis(StoppingControllerRobot, :driver)
      assert is_pid(controller_pid)

      ref = Process.monitor(controller_pid)

      :ok = BB.Safety.arm(StoppingControllerRobot)
      :ok = BB.Safety.disarm(StoppingControllerRobot)

      assert_receive {:DOWN, ^ref, :process, ^controller_pid, :disarmed}, 1_000
    end
  end

  defp safe_unregister(name) do
    Process.unregister(name)
  rescue
    ArgumentError -> :ok
  end
end
