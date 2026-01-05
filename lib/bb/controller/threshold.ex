# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.Threshold do
  @moduledoc """
  Convenience wrapper around PatternMatch for threshold monitoring.

  Transforms threshold options into a PatternMatch configuration and
  delegates all message handling to PatternMatch.

  ## Options

  - `:topic` - PubSub topic path to subscribe to (required)
  - `:field` - Field path to extract from message payload (required)
  - `:min` - Minimum acceptable value (at least one of min/max required)
  - `:max` - Maximum acceptable value (at least one of min/max required)
  - `:action` - Action to trigger when threshold exceeded (required)
  - `:cooldown_ms` - Minimum ms between triggers (default: 1000)

  ## Example

      controller :over_current, {BB.Controller.Threshold,
        topic: [:sensor, :servo_status],
        field: :current,
        max: 1.21,
        action: command(:disarm)
      }

  This is equivalent to:

      controller :over_current, {BB.Controller.PatternMatch,
        topic: [:sensor, :servo_status],
        match: fn msg -> Map.get(msg.payload, :current) > 1.21 end,
        action: command(:disarm)
      }
  """

  use BB.Controller,
    options_schema: [
      topic: [type: {:list, :atom}, required: true, doc: "PubSub topic path to subscribe to"],
      field: [
        type: {:or, [:atom, {:list, :atom}]},
        required: true,
        doc: "Field path to extract from message payload"
      ],
      min: [type: :float, doc: "Minimum acceptable value"],
      max: [type: :float, doc: "Maximum acceptable value"],
      action: [type: :any, required: true, doc: "Action to trigger when threshold exceeded"],
      cooldown_ms: [
        type: :non_neg_integer,
        default: 1000,
        doc: "Minimum milliseconds between triggers"
      ]
    ]

  alias BB.Controller.PatternMatch

  @impl BB.Controller
  def init(opts) do
    min = opts[:min]
    max = opts[:max]

    unless min || max do
      raise ArgumentError, "BB.Controller.Threshold requires at least one of :min or :max"
    end

    match_fn = build_match_fn(opts[:field], min, max)
    pattern_opts = Keyword.put(opts, :match, match_fn)

    PatternMatch.init(pattern_opts)
  end

  defdelegate handle_info(msg, state), to: PatternMatch

  defp build_match_fn(field, min, max) do
    fn msg ->
      value = get_field(msg.payload, field)
      threshold_exceeded?(value, min, max)
    end
  end

  defp get_field(payload, field) when is_atom(field), do: Map.get(payload, field)
  defp get_field(payload, path) when is_list(path), do: get_in(payload, path)

  defp threshold_exceeded?(nil, _min, _max), do: false

  defp threshold_exceeded?(value, min, max) do
    below_min? = if min, do: value < min, else: false
    above_max? = if max, do: value > max, else: false
    below_min? or above_max?
  end
end
