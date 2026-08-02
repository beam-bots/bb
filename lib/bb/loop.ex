# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Loop do
  @moduledoc """
  Timing and health accounting for periodic components.

  Components that run a periodic loop - controllers, policy runners, hardware
  bus managers - each need the same three things: a tick that doesn't drift, a
  measured time delta to hand to whatever algorithm they're driving, and some
  way to tell an operator the loop isn't keeping up. `BB.Loop` is a struct you
  embed in your component's state that provides all three.

  It is deliberately *not* a behaviour or a process. Your component stays a
  plain `BB.Controller` (or `GenServer`, or whatever); the loop is a value it
  threads through its own callbacks.

  ## Clock sources

  A loop is clocked one of two ways, chosen at `new/2`:

  - `{:rate, hertz}` - the loop schedules its own `:tick` messages. Use this
    when output must be produced on a fixed cadence regardless of input.
  - `:external` - something else decides when to step, typically the arrival of
    a message. The loop does no scheduling and only does the delta and health
    accounting. Use this when a loop's natural clock is its input; a control
    loop fed by a sensor is almost always better clocked by that sensor than by
    an independent timer.

  ## Rate-clocked usage

      def init(opts) do
        bb = Keyword.fetch!(opts, :bb)
        loop = BB.Loop.new(bb, clock: {:rate, ~u(100 hertz)})
        {:ok, %{bb: bb, loop: BB.Loop.arm(loop)}}
      end

      def handle_info(:tick, state) do
        {dt, skipped, loop} = BB.Loop.tick(state.loop)
        {:noreply, step(%{state | loop: loop}, dt, skipped)}
      end

      def terminate(_reason, state) do
        BB.Loop.cancel(state.loop)
        :ok
      end

  `tick/1` re-arms the timer itself, so there is one call per tick rather than a
  `tick`/`schedule` pair.

  ## Externally-clocked usage

  Call `observe/2` with the monotonic timestamp of whatever triggered the step -
  for a `BB.Message`, its `:monotonic_time`, so that the delta reflects the
  sensor's own sampling interval rather than when the message was dequeued:

      def handle_info({:bb, _topic, %BB.Message{} = message}, state) do
        {dt, _skipped, loop} = BB.Loop.observe(state.loop, message.monotonic_time)
        {:noreply, step(%{state | loop: loop}, dt, 0)}
      end

  `arm/1` and `cancel/1` are no-ops on an externally-clocked loop, so components
  supporting both clocks don't need to branch on which they were given.

  ## The first delta, and deltas that don't advance

  `dt` is `nil` on a loop's first tick, because there is no previous tick to
  measure from. It is also `nil` when an `observe/2` timestamp doesn't advance
  the clock, which is what a duplicate or reordered message looks like; the loop
  keeps its previous timestamp in that case so the *next* message still measures
  a correct interval.

  Both cases mean "there is no valid interval here". Match on it and skip the
  step - handing a `nil` or negative `dt` to an integrator or a derivative term
  is exactly the kind of thing this module exists to prevent:

      defp step(state, nil, _skipped), do: state
      defp step(state, dt, _skipped), do: # ... real work

  ## Missed deadlines

  A rate-clocked loop schedules against an absolute monotonic deadline that
  accumulates in nanoseconds, so a slow handler doesn't push the schedule later
  and rounding doesn't compound.

  When a handler overruns, whole missed periods are **skipped**, not queued.
  This is the important part: re-arming to a deadline that has already passed
  makes `Process.send_after/4` fire immediately, and a loop that has fallen ten
  periods behind would otherwise deliver ten back-to-back ticks with a `dt` of
  effectively zero. For anything with an integral or derivative term that is far
  worse than missing the ticks outright.

  The number of periods dropped is returned from `tick/1` and reported as the
  `:skipped` telemetry measurement. It is the loop's overrun metric: a loop that
  is consistently skipping is configured faster than the machine can actually
  run it.

  ## Achievable rates

  `Process.send_after/4` has millisecond resolution, so a period below 1ms
  cannot be represented and rates much above 1kHz will skip most of their
  periods. Well below that ceiling the BEAM's scheduling tail dominates: on a
  general-purpose kernel, tick latency has a floor of tens of milliseconds
  regardless of the period asked for, so a loop nominally at 500Hz may deliver
  half that. This is not a reason to avoid high rates, but it is a reason to
  watch `:skipped` rather than trust the configured rate.

  ## Telemetry

  Each tick with a valid `dt` emits `[:bb, :loop, :tick]`:

  - Measurements: `%{dt: float_seconds, skipped: integer, deadline_error: integer_ns}`
  - Metadata: `%{robot: module, path: [atom], clock: {:rate, float} | :external}`

  `:deadline_error` is how late the tick was against its scheduled deadline. It
  and `:skipped` are always `0` for externally-clocked loops, which have no
  deadline of their own. No event is emitted when `dt` is `nil`, since there is
  no interval to report.

  ## What this does not cover

  `BB.Loop` answers "is this loop meeting its own deadline?". It knows nothing
  about a component's inputs, so input freshness is not its job - a component
  that must stop acting on a stale sensor reading owns that timeout itself, and
  should report it separately via `BB.Diagnostic`.
  """

  alias BB.Robot.Units

  @typedoc """
  How a loop is clocked. Normalised to `{:rate, float}` in hertz, or `:external`.
  """
  @type clock :: {:rate, float()} | :external

  @typedoc """
  A `%{robot: module, path: [atom]}` map, as injected into component options.
  """
  @type bb :: %{robot: module(), path: [atom()]}

  @type t :: %__MODULE__{
          bb: bb(),
          clock: clock(),
          period_ns: pos_integer() | nil,
          deadline_ns: integer(),
          last_ns: integer() | nil,
          tick_ref: reference() | nil,
          ticks: non_neg_integer(),
          skipped: non_neg_integer()
        }

  @enforce_keys [:bb, :clock, :deadline_ns]
  defstruct [
    :bb,
    :clock,
    :period_ns,
    :deadline_ns,
    :last_ns,
    :tick_ref,
    ticks: 0,
    skipped: 0
  ]

  @tick_message :tick
  @ns_per_second 1_000_000_000
  @ns_per_millisecond 1_000_000

  @doc """
  Build a loop for a component.

  `bb` is the `%{robot: _, path: _}` map injected into component options.

  ## Options

  - `:clock` (required) - `{:rate, hertz}` or `:external`. A rate may be given
    as a `Localize.Unit` in any frequency unit (`~u(100 hertz)`) or as a plain
    positive number of hertz.

  Building a loop does not start it; call `arm/1` once the component is ready to
  receive ticks.

  ## Examples

      BB.Loop.new(bb, clock: {:rate, ~u(100 hertz)})
      BB.Loop.new(bb, clock: {:rate, 100})
      BB.Loop.new(bb, clock: :external)
  """
  @spec new(bb(), keyword()) :: t()
  def new(bb, opts) do
    clock = Keyword.fetch!(opts, :clock)

    %__MODULE__{
      bb: bb,
      clock: normalise_clock(clock),
      period_ns: period_ns(clock),
      deadline_ns: System.monotonic_time(:nanosecond)
    }
  end

  @doc """
  Schedule the loop's first tick.

  Call once from `init/1`. Subsequent ticks are armed by `tick/1`. A no-op on an
  externally-clocked loop.
  """
  @spec arm(t()) :: t()
  def arm(%__MODULE__{clock: :external} = loop), do: loop

  def arm(%__MODULE__{} = loop) do
    {loop, _skipped} = schedule(loop, System.monotonic_time(:nanosecond))
    loop
  end

  @doc """
  Record a tick on a rate-clocked loop and schedule the next one.

  Returns `{dt, skipped, loop}` where `dt` is the seconds elapsed since the
  previous tick (`nil` on the first), and `skipped` is the number of whole
  periods dropped because the loop had fallen behind.

  Call this at the top of your `:tick` handler. The next deadline is absolute,
  so arming before doing the tick's work is correct - the work's duration
  doesn't move the schedule, and an overrun is reported by the following tick.
  """
  @spec tick(t()) :: {float() | nil, non_neg_integer(), t()}
  def tick(%__MODULE__{clock: {:rate, _}} = loop) do
    now = System.monotonic_time(:nanosecond)
    dt = elapsed(loop.last_ns, now)
    deadline_error = now - loop.deadline_ns

    {loop, skipped} =
      schedule(%{loop | last_ns: now, ticks: loop.ticks + 1}, now)

    emit(loop, dt, skipped, deadline_error)
    {dt, skipped, loop}
  end

  @doc """
  Record a step on an externally-clocked loop at the given monotonic timestamp.

  `at_ns` should be the monotonic nanosecond timestamp of whatever triggered the
  step - for a `BB.Message`, its `:monotonic_time` field.

  Returns `{dt, 0, loop}`. `dt` is `nil` for the first observation, and for any
  timestamp that doesn't advance past the previous one; see the module docs.
  """
  @spec observe(t(), integer()) :: {float() | nil, 0, t()}
  def observe(%__MODULE__{clock: :external, last_ns: nil} = loop, at_ns)
      when is_integer(at_ns) do
    {nil, 0, %{loop | last_ns: at_ns, ticks: loop.ticks + 1}}
  end

  def observe(%__MODULE__{clock: :external, last_ns: last_ns} = loop, at_ns)
      when is_integer(at_ns) and at_ns > last_ns do
    dt = (at_ns - last_ns) / @ns_per_second
    loop = %{loop | last_ns: at_ns, ticks: loop.ticks + 1}
    emit(loop, dt, 0, 0)
    {dt, 0, loop}
  end

  def observe(%__MODULE__{clock: :external} = loop, at_ns) when is_integer(at_ns) do
    {nil, 0, loop}
  end

  @doc """
  Cancel any pending tick.

  Call from `terminate/2`. A no-op on an externally-clocked loop, or one that
  was never armed.
  """
  @spec cancel(t()) :: t()
  def cancel(%__MODULE__{tick_ref: nil} = loop), do: loop

  def cancel(%__MODULE__{tick_ref: tick_ref} = loop) do
    Process.cancel_timer(tick_ref)
    %{loop | tick_ref: nil}
  end

  defp schedule(%__MODULE__{period_ns: period_ns} = loop, now) do
    {deadline_ns, skipped} = next_deadline(loop.deadline_ns + period_ns, now, period_ns, 0)

    loop = %{
      loop
      | deadline_ns: deadline_ns,
        skipped: loop.skipped + skipped,
        tick_ref: Process.send_after(self(), @tick_message, ceil_ms(deadline_ns), abs: true)
    }

    {loop, skipped}
  end

  defp next_deadline(deadline_ns, now, period_ns, skipped) when deadline_ns <= now,
    do: next_deadline(deadline_ns + period_ns, now, period_ns, skipped + 1)

  defp next_deadline(deadline_ns, _now, _period_ns, skipped), do: {deadline_ns, skipped}

  defp elapsed(nil, _now), do: nil
  defp elapsed(last_ns, now), do: (now - last_ns) / @ns_per_second

  defp emit(_loop, nil, _skipped, _deadline_error), do: :ok

  defp emit(loop, dt, skipped, deadline_error) do
    :telemetry.execute(
      [:bb, :loop, :tick],
      %{dt: dt, skipped: skipped, deadline_error: deadline_error},
      %{robot: loop.bb.robot, path: loop.bb.path, clock: loop.clock}
    )
  end

  defp normalise_clock(:external), do: :external
  defp normalise_clock({:rate, rate}), do: {:rate, hertz(rate)}

  defp period_ns(:external), do: nil
  defp period_ns({:rate, rate}), do: round(@ns_per_second / hertz(rate))

  defp hertz(%Localize.Unit{} = rate) do
    rate
    |> Localize.Unit.convert!("hertz")
    |> Units.extract_float()
    |> hertz()
  end

  defp hertz(rate) when is_number(rate) and rate > 0, do: rate / 1

  # Erlang monotonic time is routinely a large negative integer, for which
  # `div/2` truncates towards zero rather than flooring - so the obvious
  # `div(ns + 999_999, 1_000_000)` rounds the wrong way for half the timeline.
  defp ceil_ms(ns), do: -Integer.floor_div(-ns, @ns_per_millisecond)
end
