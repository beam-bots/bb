# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule BB.Igniter.Transmission do
    @moduledoc """
    Shared Igniter upgrade logic for lifting actuator-driver `reverse?` options
    into per-attachment `transmission` blocks.

    Used by the upgrade tasks in each driver package (Feetech, Robotis,
    PCA9685, Pigpio). The transformation is:

    Before:

        joint :shoulder do
          type :revolute
          limit do
            lower(~u(-10 degree))
            upper(~u(190 degree))
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end
          actuator :motor, {BB.Servo.Feetech.Actuator,
            servo_id: 1, controller: :feetech, reverse?: true
          }
          link :arm
        end

    After (with `lift_offset?: true`):

        joint :shoulder do
          type :revolute
          limit do
            lower(~u(-10 degree))
            upper(~u(190 degree))
            effort(~u(10 newton_meter))
            velocity(~u(180 degree_per_second))
          end
          actuator :motor, {BB.Servo.Feetech.Actuator,
            servo_id: 1, controller: :feetech
          } do
            transmission do
              offset(~u(90.0 degree))
              reversed? true
            end
          end
          link :arm
        end

    The `offset` is computed as `(lower + upper) / 2`, preserving the
    auto-centering behaviour the Feetech/Robotis drivers used to derive
    internally. `lift_offset?: false` skips the offset computation — used by
    the PCA9685 and Pigpio upgraders, which never auto-centered.

    Also handles re-running on code that was previously upgraded to put the
    `transmission` block at the joint level: such blocks are removed and
    merged into the actuator's own block.
    """

    alias Sourceror.Zipper

    @doc """
    Lift `reverse?` opts on the given driver's actuator child-specs into
    per-attachment `transmission` blocks across the project.

    Options:

      * `:lift_offset?` (default `false`) — when true, also compute and emit
        an `offset(...)` value derived from the enclosing joint's `lower` and
        `upper` limits, preserving the implicit centering that Feetech and
        Robotis drivers used to do internally.
    """
    @spec lift_reverse_question(Igniter.t(), module(), keyword()) :: Igniter.t()
    def lift_reverse_question(igniter, driver_module, opts \\ []) do
      lift_offset? = Keyword.get(opts, :lift_offset?, false)

      {igniter, modules} =
        Igniter.Project.Module.find_all_matching_modules(igniter, fn _mod, _zipper -> true end)

      Enum.reduce(modules, igniter, &lift_in_module(&1, &2, driver_module, lift_offset?))
    end

    defp lift_in_module(module, igniter, driver_module, lift_offset?) do
      updater = fn zipper -> {:ok, update_module(zipper, driver_module, lift_offset?)} end

      case Igniter.Project.Module.find_and_update_module(igniter, module, updater) do
        {:ok, ig} -> ig
        {:error, ig} -> ig
      end
    end

    defp update_module(zipper, driver, lift_offset?) do
      current = Zipper.node(zipper)
      new_node = Macro.prewalk(current, &transform_joint(&1, driver, lift_offset?))
      Zipper.replace(zipper, new_node)
    end

    defp transform_joint({:joint, meta, [name, do_block]} = node, driver, lift_offset?)
         when is_list(do_block) do
      case do_block_body(do_block) do
        {:ok, body, wrapper} ->
          updated_body = rewrite_joint_body(body, driver, lift_offset?)
          {:joint, meta, [name, rebuild_do(wrapper, updated_body)]}

        :error ->
          node
      end
    end

    defp transform_joint(other, _driver, _lift_offset?), do: other

    defp do_block_body([{{:__block__, _, [:do]}, body} = pair]), do: {:ok, body, {:block, pair}}
    defp do_block_body(do: body), do: {:ok, body, :plain}
    defp do_block_body(_), do: :error

    defp rebuild_do({:block, {key, _}}, body), do: [{key, body}]
    defp rebuild_do(:plain, body), do: [do: body]

    defp rewrite_joint_body({:__block__, meta, stmts}, driver, lift_offset?) do
      new_stmts = do_rewrite_joint_body(stmts, driver, lift_offset?)
      {:__block__, meta, new_stmts}
    end

    defp rewrite_joint_body(stmt, driver, lift_offset?) do
      case do_rewrite_joint_body([stmt], driver, lift_offset?) do
        [single] -> single
        many -> {:__block__, [], many}
      end
    end

    defp do_rewrite_joint_body(stmts, driver, lift_offset?) do
      limits = collect_limits(stmts)

      {stmts, existing_transmission} = extract_joint_level_transmission(stmts)

      Enum.map(stmts, fn stmt ->
        rewrite_actuator(stmt, driver, lift_offset?, limits, existing_transmission)
      end)
    end

    defp collect_limits(stmts) do
      Enum.find_value(stmts, fn
        {:limit, _, [[{{:__block__, _, [:do]}, body}]]} -> extract_limits(body)
        {:limit, _, [[do: body]]} -> extract_limits(body)
        _ -> nil
      end)
    end

    # A `transmission do ... end` sibling of the actuator inside a joint is a
    # leftover from the previous (joint-level) version of this upgrader.
    # Capture and remove it so we can merge its contents into the actuator's
    # own block.
    defp extract_joint_level_transmission(stmts) do
      {kept, captured} =
        Enum.reduce(stmts, {[], nil}, fn
          {:transmission, _, [[{{:__block__, _, [:do]}, body}]]} = stmt, {acc, nil} ->
            {acc, %{body: body, ast: stmt}}

          {:transmission, _, [[do: body]]} = stmt, {acc, nil} ->
            {acc, %{body: body, ast: stmt}}

          stmt, {acc, captured} ->
            {[stmt | acc], captured}
        end)

      {Enum.reverse(kept), captured}
    end

    defp rewrite_actuator(
           {:actuator, meta, [name, child_spec]},
           driver,
           lift_offset?,
           limits,
           existing
         ) do
      case strip_reverse_question(child_spec, driver) do
        {:matched, was_true?, new_child_spec} ->
          transmission =
            build_transmission(was_true?, lift_offset?, limits, existing)

          if transmission do
            {:actuator, meta, [name, new_child_spec, [do: transmission]]}
          else
            {:actuator, meta, [name, new_child_spec]}
          end

        :unmatched ->
          {:actuator, meta, [name, child_spec]}
      end
    end

    defp rewrite_actuator(other, _driver, _lift_offset?, _limits, _existing), do: other

    defp build_transmission(reversed?, lift_offset?, limits, existing) do
      offset_line = build_offset_line(lift_offset?, limits, existing)
      reversed_line = build_reversed_line(reversed?, existing)
      reduction_line = existing && extract_reduction_line(existing.body)

      [reduction_line, offset_line, reversed_line]
      |> Enum.reject(&is_nil/1)
      |> render_transmission_source()
    end

    defp build_offset_line(lift_offset?, limits, existing) do
      with nil <- existing && extract_offset_line(existing.body) do
        if lift_offset?, do: offset_line_from_limits(limits)
      end
    end

    defp build_reversed_line(true, _existing), do: "reversed? true"

    defp build_reversed_line(false, existing) do
      if existing_has_reversed_true?(existing), do: "reversed? true"
    end

    defp render_transmission_source([]), do: nil

    defp render_transmission_source(lines) do
      Sourceror.parse_string!("transmission do\n  " <> Enum.join(lines, "\n  ") <> "\nend")
    end

    defp existing_has_reversed_true?(nil), do: false

    defp existing_has_reversed_true?(%{body: body}) do
      Enum.any?(body_stmts(body), fn
        {:reversed?, _, [true]} -> true
        {:reversed?, _, [{:__block__, _, [true]}]} -> true
        _ -> false
      end)
    end

    defp extract_offset_line(body) do
      Enum.find_value(body_stmts(body), fn
        {:offset, _, [arg]} -> "offset #{Macro.to_string(arg)}"
        _ -> nil
      end)
    end

    defp extract_reduction_line(body) do
      Enum.find_value(body_stmts(body), fn
        {:reduction, _, [arg]} -> "reduction #{Macro.to_string(arg)}"
        _ -> nil
      end)
    end

    defp body_stmts({:__block__, _, stmts}), do: stmts
    defp body_stmts(stmt), do: [stmt]

    # Look for `{Driver, opts}` (or `{Driver, opts}` aliased) and strip `reverse?:`.
    defp strip_reverse_question({:{}, meta, [driver_alias, opts]}, driver) do
      strip_from_tuple(driver_alias, opts, driver, fn new_opts ->
        {:{}, meta, [driver_alias, new_opts]}
      end)
    end

    defp strip_reverse_question({driver_alias, opts}, driver) do
      strip_from_tuple(driver_alias, opts, driver, fn new_opts ->
        {driver_alias, new_opts}
      end)
    end

    defp strip_reverse_question({:__block__, meta, [inner]}, driver) do
      case strip_reverse_question(inner, driver) do
        {:matched, was_true?, new_inner} -> {:matched, was_true?, {:__block__, meta, [new_inner]}}
        :unmatched -> :unmatched
      end
    end

    defp strip_reverse_question(_, _), do: :unmatched

    defp strip_from_tuple(driver_alias, opts, driver, rebuild) do
      if alias_matches?(driver_alias, driver) and is_list(opts) do
        case pop_reverse_question(opts) do
          :not_present ->
            {:matched, false, rebuild.(opts)}

          {value, new_opts} ->
            was_true? = literal_true?(value)
            {:matched, was_true?, rebuild.(new_opts)}
        end
      else
        :unmatched
      end
    end

    defp alias_matches?({:__aliases__, _, parts}, driver_module) do
      Module.concat(parts) == driver_module
    end

    defp alias_matches?(_, _), do: false

    defp pop_reverse_question(opts) do
      case Enum.split_with(opts, fn
             {{:__block__, _, [:reverse?]}, _} -> true
             {:reverse?, _} -> true
             _ -> false
           end) do
        {[], _} ->
          :not_present

        {[{_, value} | _], rest} ->
          {value, rest}
      end
    end

    defp literal_true?({:__block__, _, [true]}), do: true
    defp literal_true?(true), do: true
    defp literal_true?(_), do: false

    defp extract_limits({:__block__, _, stmts}), do: extract_limits(stmts)

    defp extract_limits(stmts) when is_list(stmts) do
      Enum.reduce(stmts, %{lower: nil, upper: nil}, fn
        {:lower, _, [value]}, acc -> %{acc | lower: value}
        {:upper, _, [value]}, acc -> %{acc | upper: value}
        _, acc -> acc
      end)
    end

    defp extract_limits(_), do: nil

    defp offset_line_from_limits(nil), do: nil
    defp offset_line_from_limits(%{lower: nil}), do: nil
    defp offset_line_from_limits(%{upper: nil}), do: nil

    defp offset_line_from_limits(%{lower: lower_ast, upper: upper_ast}) do
      with {lower_value, unit} <- unit_literal(lower_ast),
           {upper_value, ^unit} <- unit_literal(upper_ast),
           midpoint when midpoint != 0.0 <- (lower_value + upper_value) / 2 do
        "offset ~u(#{format_number(midpoint)} #{unit})"
      else
        _ -> nil
      end
    end

    defp unit_literal({:sigil_u, _, [{:<<>>, _, [text]}, _]}) when is_binary(text) do
      with [num_str, unit] <- String.split(text, " ", parts: 2),
           {value, ""} <- parse_number(num_str) do
        {value, unit}
      else
        _ -> nil
      end
    end

    defp unit_literal(_), do: nil

    defp parse_number(str) do
      case Float.parse(str) do
        {_, ""} = ok -> ok
        _ -> integer_to_float(str)
      end
    end

    defp integer_to_float(str) do
      case Integer.parse(str) do
        {value, ""} -> {value * 1.0, ""}
        other -> other
      end
    end

    defp format_number(n) when is_float(n) do
      if n == Float.round(n, 0), do: :erlang.float_to_binary(n, decimals: 1), else: to_string(n)
    end
  end
end
