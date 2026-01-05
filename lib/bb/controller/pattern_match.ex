# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.PatternMatch do
  @moduledoc """
  Controller that triggers an action when a message matches a predicate.

  This is the base reactive controller implementation. Other reactive controllers
  like `BB.Controller.Threshold` are convenience wrappers around this module.

  ## Options

  - `:topic` - PubSub topic path to subscribe to (required)
  - `:match` - Predicate function `fn msg -> boolean end` (required)
  - `:action` - Action to trigger on match (required)
  - `:cooldown_ms` - Minimum ms between triggers (default: 1000)

  ## Example

      controller :collision, {BB.Controller.PatternMatch,
        topic: [:sensor, :proximity],
        match: fn msg -> msg.payload.distance < 0.05 end,
        action: command(:disarm)
      }
  """

  use BB.Controller,
    options_schema: [
      topic: [type: {:list, :atom}, required: true, doc: "PubSub topic path to subscribe to"],
      match: [type: {:fun, 1}, required: true, doc: "Predicate function fn msg -> boolean"],
      action: [type: :any, required: true, doc: "Action to trigger on match"],
      cooldown_ms: [
        type: :non_neg_integer,
        default: 1000,
        doc: "Minimum milliseconds between triggers"
      ]
    ]

  alias BB.Controller.Action
  alias BB.Controller.Action.Context
  alias BB.Robot.Runtime

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    BB.subscribe(bb.robot, opts[:topic])

    {:ok,
     %{
       opts: opts,
       last_triggered: :never
     }}
  end

  @impl BB.Controller
  def handle_info({:bb, _path, %BB.Message{} = msg}, state) do
    opts = state.opts

    if opts[:match].(msg) and cooldown_elapsed?(state) do
      context = build_context(opts)
      Action.execute(opts[:action], msg, context)
      {:noreply, %{state | last_triggered: System.monotonic_time(:millisecond)}}
    else
      {:noreply, state}
    end
  end

  def handle_info(_msg, state), do: {:noreply, state}

  defp cooldown_elapsed?(%{last_triggered: :never}), do: true

  defp cooldown_elapsed?(state) do
    now = System.monotonic_time(:millisecond)
    now - state.last_triggered >= state.opts[:cooldown_ms]
  end

  defp build_context(opts) do
    bb = opts[:bb]

    %Context{
      robot_module: bb.robot,
      robot: bb.robot.robot(),
      robot_state: Runtime.state(bb.robot),
      controller_name: bb.path |> List.last()
    }
  end
end
