# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule BB.Igniter.Transmission do
    @moduledoc """
    Shared Igniter upgrade logic for lifting actuator-driver `reverse?` options
    into joint-level `transmission` blocks.

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
          transmission do
            offset(~u(90.0 degree))
            reversed? true
          end
          actuator :motor, {BB.Servo.Feetech.Actuator,
            servo_id: 1, controller: :feetech
          }
          link :arm
        end

    The `offset` is computed as `(lower + upper) / 2`, preserving the
    auto-centering behaviour the Feetech/Robotis drivers used to derive
    internally. `lift_offset?: false` skips the offset computation — used by
    the PCA9685 and Pigpio upgraders, which never auto-centered.
    """

    alias Sourceror.Zipper

    @doc """
    Lift `reverse?` opts on the given driver's actuator child-specs into
    joint-level `transmission` blocks across the project.

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

    # Apply the transmission lift to every `joint :name do ... end` block in
    # the module body. We work directly on the quoted form rather than the
    # zipper since we need recursive transformation of nested calls.
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

    # The joint's `do` body can be a single call or a `:__block__` of calls.
    # Normalise to a list, transform, and pack back.
    defp rewrite_joint_body({:__block__, meta, stmts}, driver, lift_offset?) do
      {new_stmts, joint_state} = scan_and_modify(stmts, driver, lift_offset?)
      new_stmts = insert_transmission(new_stmts, joint_state)
      {:__block__, meta, new_stmts}
    end

    defp rewrite_joint_body(stmt, driver, lift_offset?) do
      {new_stmts, joint_state} = scan_and_modify([stmt], driver, lift_offset?)
      new_stmts = insert_transmission(new_stmts, joint_state)

      case new_stmts do
        [single] -> single
        many -> {:__block__, [], many}
      end
    end

    # Walk statements in the joint body. For each one:
    #   - actuator(...) calls matching driver: strip `reverse?:` opt, capture
    #     whether it was true.
    #   - limit do ... end blocks: capture lower/upper for offset computation.
    #   - transmission do ... end blocks: capture so we can merge into the
    #     existing block instead of inserting a new one.
    defp scan_and_modify(stmts, driver, lift_offset?) do
      Enum.map_reduce(
        stmts,
        %{
          reverse?: false,
          touched_actuator?: false,
          existing_transmission_index: nil,
          limits: nil,
          lift_offset?: lift_offset?
        },
        fn stmt, acc ->
          process_stmt(stmt, driver, acc)
        end
      )
    end

    defp process_stmt({:actuator, meta, [name, child_spec]}, driver, acc) do
      case strip_reverse_question(child_spec, driver) do
        {:matched, was_true?, new_child_spec} ->
          new_acc = %{acc | reverse?: acc.reverse? or was_true?, touched_actuator?: true}
          {{:actuator, meta, [name, new_child_spec]}, new_acc}

        :unmatched ->
          {{:actuator, meta, [name, child_spec]}, acc}
      end
    end

    defp process_stmt({:limit, _meta, [[{{:__block__, _, [:do]}, body}]]} = stmt, _driver, acc) do
      {stmt, %{acc | limits: extract_limits(body)}}
    end

    defp process_stmt({:limit, _meta, [[do: body]]} = stmt, _driver, acc) do
      {stmt, %{acc | limits: extract_limits(body)}}
    end

    defp process_stmt(stmt, _driver, acc), do: {stmt, acc}

    # Look for `{Driver, opts}` (or `{Driver, opts}` aliased) and strip `reverse?:`.
    defp strip_reverse_question(
           {:{}, meta, [driver_alias, opts]},
           driver
         ) do
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
            :unmatched

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

    # Pull `lower` and `upper` calls out of a limit body so we can compute an offset.
    defp extract_limits({:__block__, _, stmts}), do: extract_limits(stmts)

    defp extract_limits(stmts) when is_list(stmts) do
      Enum.reduce(stmts, %{lower: nil, upper: nil}, fn
        {:lower, _, [value]}, acc -> %{acc | lower: value}
        {:upper, _, [value]}, acc -> %{acc | upper: value}
        _, acc -> acc
      end)
    end

    defp extract_limits(_), do: nil

    defp insert_transmission(stmts, %{touched_actuator?: false}), do: stmts

    defp insert_transmission(stmts, %{reverse?: false, lift_offset?: false}), do: stmts

    defp insert_transmission(stmts, state) do
      case transmission_source(state) do
        nil ->
          stmts

        source ->
          new_node = Sourceror.parse_string!(source)
          place_after_limit(stmts, new_node)
      end
    end

    defp transmission_source(state) do
      offset_line =
        if state.lift_offset? do
          offset_line_from_limits(state.limits)
        end

      reversed_line =
        if state.reverse? do
          "reversed? true"
        end

      case Enum.reject([offset_line, reversed_line], &is_nil/1) do
        [] ->
          nil

        lines ->
          "transmission do\n  " <> Enum.join(lines, "\n  ") <> "\nend"
      end
    end

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

    # Match `~u(-110 degree)` / `~u(190 degree)` / etc. and pull out the numeric
    # value and unit name. Returns nil for non-literal expressions (param refs,
    # arithmetic, etc.) so the upgrader gives up rather than guessing.
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

    # Place a new node immediately after the `limit do ... end` call if one is
    # present, otherwise prepend it.
    defp place_after_limit(stmts, new_node) do
      case Enum.find_index(stmts, &limit_call?/1) do
        nil -> [new_node | stmts]
        idx -> List.insert_at(stmts, idx + 1, new_node)
      end
    end

    defp limit_call?({:limit, _, _}), do: true
    defp limit_call?(_), do: false
  end
end
