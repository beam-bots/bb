# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.Source do
  @moduledoc """
  Where an estimator's inputs come from.

  `BB.Estimator.Server` doesn't receive inputs itself; it asks a source for
  them. The default, `BB.Estimator.Source.PubSub`, subscribes to a path and
  treats every `%BB.Message{}` published there as an input. A sensor whose
  readings don't travel over `BB.PubSub` names another source with
  `c:BB.Sensor.estimator_source/0`, and the estimators nested inside it use
  that instead.

  The server owns the pace. It calls `c:request/1` once it is ready for an
  input - after it has started, and again each time it has finished with what
  the source last gave it - so a source that delivers by demand never hands an
  estimator more than it has asked for. A source that can't be paced, such as
  PubSub, makes `c:request/1` a no-op.

  The server requests again only for an input or a `:spent` event, so a source
  must tell the message that answers a request from one that merely
  accompanies it. Requesting on a message that arrives alongside an input -
  ahead of it in the mailbox - would leave a second request outstanding by the
  time the input is dispatched, and the estimator would be handed an input it
  hadn't yet asked for.
  """

  @typedoc "Whatever a source needs to recognise its own messages. Opaque to the server."
  @type state :: term()

  @typedoc """
  What a message received by the estimator's process means to a source.

  - `{:input, input, monotonic_time, node}` - an input, handed to
    `c:BB.Estimator.handle_input/2` as `input`. `monotonic_time` is when it
    was produced, in `System.monotonic_time(:nanosecond)` on `node`; the
    server measures `max_input_age` and latency from it, and refuses an input
    from another node because monotonic clocks are node-local.
  - `:no_input` - the source's own message, carrying no input and leaving
    the outstanding request where it was. The server does nothing with it.
  - `:spent` - the source's own message, which answered the outstanding
    request without an input - a stream pausing, say. The server requests
    again.
  - `{:ended, reason}` - the source will deliver nothing more. The server
    stops, and its supervisor starts it again with a fresh subscription.
  - `nil` - not this source's message.
  """
  @type event ::
          {:input, input :: term(), monotonic_time :: integer(), node()}
          | :no_input
          | :spent
          | {:ended, reason :: term()}
          | nil

  @doc """
  Start delivering to the calling process the inputs at `path` on `robot`.

  `args` are whatever the sensor returned alongside the source module from
  `c:BB.Sensor.estimator_source/0`.
  """
  @callback subscribe(robot :: module(), path :: [atom()], args :: keyword()) ::
              {:ok, state()} | {:error, term()}

  @doc "What `message`, received by the subscribing process, means for this source."
  @callback input_event(state(), message :: term()) :: event()

  @doc "Ask for the next input."
  @callback request(state()) :: :ok
end
