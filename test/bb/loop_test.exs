# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.LoopTest do
  use ExUnit.Case, async: true

  import BB.Unit

  alias BB.Loop

  @bb %{robot: BB.LoopTest.Robot, path: [:base_link, :test_loop]}

  # These tests drive real timers, so they are sensitive to scheduler load. The
  # period is kept well above the BEAM's scheduling tail and receive timeouts
  # well above that again, so that a loaded CI machine fails them only for real
  # regressions. Assertions are on structural properties (a period is either
  # ticked or skipped; a delta matches wall time) rather than on hitting the
  # nominal rate, which is not something the VM guarantees.
  @rate_hz 50
  @period_ms 20
  @receive_timeout 2_000

  defp rate_loop(hertz \\ @rate_hz), do: Loop.new(@bb, clock: {:rate, hertz})
  defp external_loop, do: Loop.new(@bb, clock: :external)

  defp attach_telemetry(context) do
    handler_id = "loop-test-#{inspect(context.test)}"
    test_pid = self()

    :telemetry.attach(
      handler_id,
      [:bb, :loop, :tick],
      fn event, measurements, metadata, _config ->
        send(test_pid, {:telemetry, event, measurements, metadata})
      end,
      nil
    )

    on_exit(fn -> :telemetry.detach(handler_id) end)
    :ok
  end

  describe "new/2" do
    test "accepts a rate as a unit" do
      loop = Loop.new(@bb, clock: {:rate, ~u(100 hertz)})

      assert loop.clock == {:rate, 100.0}
      assert loop.period_ns == 10_000_000
    end

    test "accepts a rate as a plain number of hertz" do
      assert Loop.new(@bb, clock: {:rate, 100}).clock == {:rate, 100.0}
      assert Loop.new(@bb, clock: {:rate, 2.5}).period_ns == 400_000_000
    end

    test "rejects a non-positive rate" do
      assert_raise FunctionClauseError, fn -> Loop.new(@bb, clock: {:rate, 0}) end
      assert_raise FunctionClauseError, fn -> Loop.new(@bb, clock: {:rate, -1}) end
    end

    test "rejects a bb map missing robot or path" do
      # Caught here rather than on the first tick, where the telemetry metadata
      # would otherwise be the first thing to notice. Built with Map.delete/2
      # so the type checker can't narrow it: a real `bb` comes out of
      # `Keyword.fetch!/2` at runtime, where it can't be checked statically.
      for key <- [:robot, :path] do
        bb = Map.delete(@bb, key)

        assert_raise FunctionClauseError, fn -> Loop.new(bb, clock: :external) end
      end
    end

    test "an external clock has no period" do
      loop = external_loop()

      assert loop.clock == :external
      assert loop.period_ns == nil
    end

    test "does not schedule anything" do
      _loop = rate_loop()

      refute_receive :tick, 50
    end
  end

  describe "arm/1" do
    test "schedules the first tick" do
      Loop.arm(rate_loop())

      assert_receive :tick, @receive_timeout
    end

    test "is a no-op for an external clock" do
      loop = external_loop()

      assert Loop.arm(loop) == loop
      refute_receive :tick, 50
    end
  end

  describe "tick/1" do
    test "has no delta on the first tick" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout

      assert {nil, _skipped, _loop} = Loop.tick(loop)
    end

    test "measures the delta between ticks" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout
      {nil, _skipped, loop} = Loop.tick(loop)

      # Compare against wall time rather than the nominal period: scheduling
      # latency is unbounded, but the reported deltas must add up to the real
      # interval. Measuring across several ticks keeps the endpoint error from
      # dominating the tolerance.
      before = System.monotonic_time(:nanosecond)

      {deltas, _loop} =
        Enum.map_reduce(1..5, loop, fn _, loop ->
          assert_receive :tick, @receive_timeout
          {dt, _skipped, loop} = Loop.tick(loop)
          {dt, loop}
        end)

      measured = (System.monotonic_time(:nanosecond) - before) / 1_000_000_000

      assert_in_delta Enum.sum(deltas), measured, 0.02
    end

    test "re-arms the next tick" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout
      {nil, _skipped, _loop} = Loop.tick(loop)

      assert_receive :tick, @receive_timeout
    end

    test "counts ticks" do
      loop = Loop.arm(rate_loop())

      loop =
        Enum.reduce(1..3, loop, fn _, loop ->
          assert_receive :tick, @receive_timeout
          {_dt, _skipped, loop} = Loop.tick(loop)
          loop
        end)

      assert loop.ticks == 3
    end
  end

  describe "tick/1 when the handler overruns" do
    test "skips whole missed periods rather than queueing them" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout
      {nil, _skipped, loop} = Loop.tick(loop)

      # Twenty periods' worth of work in one handler.
      Process.sleep(20 * @period_ms)

      assert_receive :tick, @receive_timeout
      {dt, skipped, _loop} = Loop.tick(loop)

      assert dt >= 0.02 * 20 * 0.9,
             "expected the measured delta to reflect the overrun, got #{dt}"

      assert skipped >= 10, "expected missed periods to be reported, got #{skipped}"
    end

    test "does not deliver a burst of catch-up ticks" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout
      {nil, _skipped, loop} = Loop.tick(loop)

      Process.sleep(20 * @period_ms)

      assert_receive :tick, @receive_timeout
      {_dt, _skipped, loop} = Loop.tick(loop)

      # The next five ticks span five real periods. A loop that caught up by
      # firing its missed deadlines would deliver them back-to-back, summing to
      # roughly nothing - which is the failure absolute deadlines alone do *not*
      # prevent, since a deadline already in the past fires immediately.
      #
      # The sum is asserted rather than each delta: one short interval is
      # legitimate, because skipping forward resynchronises to the deadline grid
      # and can land the next deadline just after the current tick. Load can
      # only push this total up, never down.
      {deltas, _loop} =
        Enum.map_reduce(1..5, loop, fn _, loop ->
          assert_receive :tick, @receive_timeout
          {dt, _skipped, loop} = Loop.tick(loop)
          {dt, loop}
        end)

      assert Enum.sum(deltas) >= 4 * @period_ms / 1000,
             "expected five ticks to span five periods, got #{inspect(deltas)}"
    end

    test "accounts for every period as either a tick or a skip" do
      run_ms = 50 * @period_ms
      loop = Loop.arm(rate_loop())

      started = System.monotonic_time(:millisecond)
      loop = drain_until(loop, started + run_ms)
      elapsed = System.monotonic_time(:millisecond) - started

      # The deadline advances by exactly one period per tick-or-skip, so this
      # holds regardless of how badly the machine is keeping up. Only the
      # partial period either side of the run is unaccounted for.
      periods = elapsed / @period_ms
      accounted = loop.ticks + loop.skipped

      assert_in_delta accounted, periods, 3
    end
  end

  describe "observe/2" do
    test "has no delta on the first observation" do
      assert {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)
      assert loop.ticks == 1
    end

    test "measures the delta between observations" do
      {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)

      assert {dt, 0, _loop} = Loop.observe(loop, 3_000_000)
      assert dt == 0.002
    end

    test "never reports a skip" do
      {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)
      {_dt, skipped, _loop} = Loop.observe(loop, 100_000_000)

      assert skipped == 0
    end

    test "ignores a timestamp that does not advance" do
      {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)
      {_dt, 0, loop} = Loop.observe(loop, 2_000_000)

      assert {nil, 0, unchanged} = Loop.observe(loop, 2_000_000)
      assert unchanged == loop

      assert {nil, 0, unchanged} = Loop.observe(loop, 1_500_000)
      assert unchanged == loop
    end

    test "measures the next delta from the last advancing timestamp" do
      {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)
      {_dt, 0, loop} = Loop.observe(loop, 2_000_000)
      {nil, 0, loop} = Loop.observe(loop, 1_500_000)

      assert {dt, 0, _loop} = Loop.observe(loop, 3_000_000)
      assert dt == 0.001
    end

    test "does not schedule anything" do
      {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)
      {_dt, 0, _loop} = Loop.observe(loop, 2_000_000)

      refute_receive :tick, 50
    end
  end

  describe "cancel/1" do
    test "cancels a pending tick" do
      loop = Loop.arm(rate_loop())

      assert %{tick_ref: nil} = Loop.cancel(loop)
      refute_receive :tick, 50
    end

    test "is a no-op on an unarmed loop" do
      loop = rate_loop()

      assert Loop.cancel(loop) == loop
    end

    test "discards a tick that had already been delivered" do
      loop = Loop.arm(rate_loop())
      Process.sleep(3 * @period_ms)

      # The timer has fired and :tick is sitting in the mailbox. Left there, the
      # next arm/1 would give the loop two independent tick chains.
      Loop.cancel(loop)

      refute_receive :tick, 50
    end

    test "a re-armed loop does not tick immediately" do
      armed = Loop.arm(rate_loop())
      Process.sleep(3 * @period_ms)

      armed
      |> Loop.cancel()
      |> Loop.arm()

      # Without the discard, the stale tick would be waiting already.
      refute_receive :tick, 2
      assert_receive :tick, @receive_timeout
    end

    test "is a no-op for an external clock" do
      loop = external_loop()

      assert Loop.cancel(loop) == loop
    end
  end

  describe "telemetry" do
    setup :attach_telemetry

    test "emits a tick event with measurements and metadata" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout
      {nil, _skipped, loop} = Loop.tick(loop)

      assert_receive :tick, @receive_timeout
      {_dt, _skipped, _loop} = Loop.tick(loop)

      assert_receive {:telemetry, [:bb, :loop, :tick], measurements, metadata}

      assert %{dt: dt, skipped: skipped, deadline_error: deadline_error} = measurements
      assert is_float(dt)
      assert is_integer(skipped)
      assert is_integer(deadline_error)

      assert metadata == %{
               robot: BB.LoopTest.Robot,
               path: [:base_link, :test_loop],
               clock: {:rate, @rate_hz / 1}
             }
    end

    test "does not emit when there is no interval to report" do
      loop = Loop.arm(rate_loop())
      assert_receive :tick, @receive_timeout
      {nil, _skipped, _loop} = Loop.tick(loop)

      refute_receive {:telemetry, [:bb, :loop, :tick], _measurements, _metadata}, 50
    end

    test "emits for an externally-clocked loop" do
      {nil, 0, loop} = Loop.observe(external_loop(), 1_000_000)
      {_dt, 0, _loop} = Loop.observe(loop, 3_000_000)

      assert_receive {:telemetry, [:bb, :loop, :tick], measurements, metadata}

      assert measurements == %{dt: 0.002, skipped: 0, deadline_error: 0}
      assert metadata.clock == :external
    end
  end

  defp drain_until(loop, until_ms) do
    if System.monotonic_time(:millisecond) >= until_ms do
      loop
    else
      receive do
        :tick ->
          {_dt, _skipped, loop} = Loop.tick(loop)
          drain_until(loop, until_ms)
      after
        @receive_timeout -> loop
      end
    end
  end
end
