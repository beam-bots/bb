# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  # credo:disable-for-next-line Credo.Check.Readability.ModuleNames
  defmodule BB.Igniter.Upgrade.V0_16 do
    @moduledoc """
    Migration steps for the 0.15 → 0.16 jump.

    Run via `mix igniter.upgrade bb`. The two breaking changes covered:

      * `auto_disarm_on_error` setting was removed from the robot DSL — safety
        escalation now happens via the topology supervisor's restart budget.
      * `ex_cldr_units` replaced with `localize`; `BB.Cldr` deleted; unit struct
        is now `%Localize.Unit{}` with a `name` (string) field instead of
        `%Cldr.Unit{}` with `unit` (atom).
    """

    alias Igniter.Code.Common
    alias Igniter.Code.Function
    alias Igniter.Code.Module
    alias Igniter.Project.Module, as: ProjectModule
    alias Sourceror.Zipper

    @bb_use_macros [BB, BB.Actuator, BB.Sensor, BB.Controller, BB.Bridge, BB.Command]
    @bb_behaviours [BB.Safety]

    @doc """
    Removes any `auto_disarm_on_error <bool>` line from `settings do … end`
    blocks in modules that `use BB`.
    """
    @spec remove_auto_disarm_on_error(Igniter.t(), keyword()) :: Igniter.t()
    def remove_auto_disarm_on_error(igniter, _opts) do
      {igniter, modules} = find_modules_using(igniter, [BB])
      Enum.reduce(modules, igniter, &remove_auto_disarm_from_module/2)
    end

    defp remove_auto_disarm_from_module(module, igniter) do
      ProjectModule.find_and_update_module!(igniter, module, &remove_auto_disarm_zipper/1)
    end

    defp remove_auto_disarm_zipper(zipper) do
      with {:ok, settings_zipper} <-
             Function.move_to_function_call(zipper, :settings, 1, &has_do_block?/1),
           {:ok, do_zipper} <- Common.move_to_do_block(settings_zipper) do
        do_zipper =
          do_zipper
          |> Common.maybe_move_to_block()
          |> remove_auto_disarm_line()

        {:ok, do_zipper}
      else
        _ -> {:ok, zipper}
      end
    end

    defp remove_auto_disarm_line(zipper) do
      Common.remove(zipper, fn z ->
        Function.function_call?(z, :auto_disarm_on_error, 1)
      end)
    end

    @doc """
    Rewrites `alias BB.Cldr.Unit` to `alias BB.Unit` (preserving any `as:`
    clause) wherever it appears.
    """
    @spec rename_bb_cldr_unit_alias(Igniter.t(), keyword()) :: Igniter.t()
    def rename_bb_cldr_unit_alias(igniter, _opts) do
      {igniter, modules} =
        ProjectModule.find_all_matching_modules(igniter, fn _module, zipper ->
          alias_bb_cldr_unit?(zipper)
        end)

      Enum.reduce(modules, igniter, &rename_alias_in_module/2)
    end

    defp rename_alias_in_module(module, igniter) do
      ProjectModule.find_and_update_module!(igniter, module, fn zipper ->
        Common.update_all_matches(zipper, &alias_bb_cldr_unit_node?/1, &rewrite_alias_match/1)
      end)
    end

    defp rewrite_alias_match(z), do: {:code, rewrite_alias_node(z.node)}

    defp alias_bb_cldr_unit?(zipper) do
      match?({:ok, _}, Common.move_to(zipper, &alias_bb_cldr_unit_node?/1))
    end

    defp alias_bb_cldr_unit_node?(%Zipper{node: node}) do
      case node do
        {:alias, _, [{:__aliases__, _, [:BB, :Cldr, :Unit]}]} -> true
        {:alias, _, [{:__aliases__, _, [:BB, :Cldr, :Unit]}, _opts]} -> true
        _ -> false
      end
    end

    defp rewrite_alias_node({:alias, meta, [{:__aliases__, am, [:BB, :Cldr, :Unit]}]}) do
      {:alias, meta, [{:__aliases__, am, [:BB, :Unit]}]}
    end

    defp rewrite_alias_node({:alias, meta, [{:__aliases__, am, [:BB, :Cldr, :Unit]}, opts]}) do
      {:alias, meta, [{:__aliases__, am, [:BB, :Unit]}, opts]}
    end

    @doc """
    Rewrites `Cldr.Unit.foo(...)` calls to `Localize.Unit.foo(...)` inside
    modules that use a BB DSL macro or implement `BB.Safety`. Atom unit
    identifiers (`:newton_meter`) are converted to CLDR canonical dash form
    (`"newton-meter"`) and `Localize.Unit.new!/2` has its arguments reversed
    so the value comes first.
    """
    @spec rewrite_cldr_unit_calls(Igniter.t(), keyword()) :: Igniter.t()
    def rewrite_cldr_unit_calls(igniter, _opts) do
      {igniter, modules} = find_bb_user_modules(igniter)

      Enum.reduce(modules, igniter, fn module, igniter ->
        ProjectModule.find_and_update_module!(igniter, module, fn zipper ->
          {:ok,
           zipper
           |> rewrite_calls_of({Cldr.Unit, :new!}, 2, &rewrite_new_call/1)
           |> rewrite_calls_of({Cldr.Unit, :convert!}, 2, &rewrite_convert_call/1)
           |> rewrite_calls_of({Cldr.Unit, :convert}, 2, &rewrite_convert_call/1)
           |> rewrite_calls_of({Cldr.Unit, :compatible?}, 2, &rewrite_compatible_call/1)
           |> rewrite_calls_of({Cldr.Unit, :compare}, 2, &rewrite_module_only/1)
           |> rewrite_calls_of({Cldr.Unit, :to_string!}, [1, 2], &rewrite_module_only/1)
           |> rewrite_calls_of({Cldr.Unit, :to_string}, [1, 2], &rewrite_module_only/1)}
        end)
      end)
    end

    defp rewrite_calls_of(zipper, {module, fun}, arities, rewriter) do
      arities = List.wrap(arities)
      pred = fn z -> matches_any_arity?(z, module, fun, arities) end
      replace = fn z -> {:code, rewriter.(z.node)} end

      case Common.update_all_matches(zipper, pred, replace) do
        {:ok, z} -> z
        _ -> zipper
      end
    end

    defp matches_any_arity?(z, module, fun, arities) do
      Enum.any?(arities, fn a -> Function.function_call?(z, {module, fun}, a) end)
    end

    # Pipe form delegates to the unwrapped call form.
    defp rewrite_new_call({:|>, pm, [piped, call]}),
      do: {:|>, pm, [piped, rewrite_new_call(call)]}

    # Two-arg call: Cldr.Unit.new!(:atom, value) or Cldr.Unit.new!(value, :atom)
    defp rewrite_new_call({{:., dm, [_old_mod, :new!]}, m, [arg1, arg2]}) do
      {value, unit} = pick_value_and_unit(arg1, arg2)
      {{:., dm, [localize_unit_alias(dm), :new!]}, m, [value, unit_to_string_node(unit)]}
    end

    # Single-arg call (from pipe): Cldr.Unit.new!(:atom)
    defp rewrite_new_call({{:., dm, [_old_mod, :new!]}, m, [arg]}) do
      {{:., dm, [localize_unit_alias(dm), :new!]}, m, [unit_to_string_node(arg)]}
    end

    defp rewrite_convert_call({:|>, pm, [piped, call]}),
      do: {:|>, pm, [piped, rewrite_convert_call(call)]}

    defp rewrite_convert_call({{:., dm, [_old_mod, fun]}, m, [unit, target]}) do
      {{:., dm, [localize_unit_alias(dm), fun]}, m, [unit, unit_to_string_node(target)]}
    end

    defp rewrite_convert_call({{:., dm, [_old_mod, fun]}, m, [target]}) do
      {{:., dm, [localize_unit_alias(dm), fun]}, m, [unit_to_string_node(target)]}
    end

    defp rewrite_compatible_call({:|>, pm, [piped, call]}),
      do: {:|>, pm, [piped, rewrite_compatible_call(call)]}

    defp rewrite_compatible_call({{:., dm, [_old_mod, :compatible?]}, m, [a, b]}) do
      {{:., dm, [localize_unit_alias(dm), :compatible?]}, m, [a, unit_to_string_node(b)]}
    end

    defp rewrite_compatible_call({{:., dm, [_old_mod, :compatible?]}, m, [b]}) do
      {{:., dm, [localize_unit_alias(dm), :compatible?]}, m, [unit_to_string_node(b)]}
    end

    defp rewrite_module_only({:|>, pm, [piped, call]}),
      do: {:|>, pm, [piped, rewrite_module_only(call)]}

    # Cldr.Unit.compare(a, b) → Localize.Unit.compare(a, b) (module-only swap)
    defp rewrite_module_only({{:., dm, [_old_mod, fun]}, m, args}) do
      {{:., dm, [localize_unit_alias(dm), fun]}, m, args}
    end

    defp localize_unit_alias(meta) do
      {:__aliases__, meta, [:Localize, :Unit]}
    end

    defp pick_value_and_unit(arg1, arg2) do
      if atom_literal?(arg1), do: {arg2, arg1}, else: {arg1, arg2}
    end

    defp atom_literal?(node) when is_atom(node) and not is_nil(node), do: true
    defp atom_literal?({:__block__, _, [name]}) when is_atom(name) and not is_nil(name), do: true
    defp atom_literal?(_), do: false

    defp unit_to_string_node(name) when is_atom(name) and not is_nil(name) do
      {:__block__, [], [to_dash_string(name)]}
    end

    defp unit_to_string_node(binary) when is_binary(binary) do
      {:__block__, [], [binary]}
    end

    defp unit_to_string_node({:__block__, m, [name]}) when is_atom(name) and not is_nil(name) do
      {:__block__, m, [to_dash_string(name)]}
    end

    defp unit_to_string_node({:__block__, _, [binary]} = node) when is_binary(binary), do: node

    # Leave unrecognised shapes untouched — the user will see the result.
    defp unit_to_string_node(other), do: other

    defp to_dash_string(name) when is_atom(name) do
      name |> Atom.to_string() |> String.replace("_", "-")
    end

    @doc """
    Rewrites `%Cldr.Unit{...}` literal/pattern uses to `%Localize.Unit{...}`
    and translates field renames (`unit: :foo` → `name: "foo"`). Limited to
    modules that use a BB DSL macro or implement `BB.Safety`.
    """
    @spec rewrite_cldr_unit_struct_patterns(Igniter.t(), keyword()) :: Igniter.t()
    def rewrite_cldr_unit_struct_patterns(igniter, _opts) do
      {igniter, modules} = find_bb_user_modules(igniter)
      Enum.reduce(modules, igniter, &rewrite_struct_in_module/2)
    end

    defp rewrite_struct_in_module(module, igniter) do
      ProjectModule.find_and_update_module!(igniter, module, fn zipper ->
        Common.update_all_matches(zipper, &cldr_unit_struct?/1, &rewrite_struct_match/1)
      end)
    end

    defp rewrite_struct_match(z), do: {:code, rewrite_struct_node(z.node)}

    defp cldr_unit_struct?(%Zipper{node: {:%, _, [{:__aliases__, _, [:Cldr, :Unit]}, _fields]}}),
      do: true

    defp cldr_unit_struct?(_), do: false

    defp rewrite_struct_node({:%, m, [{:__aliases__, am, [:Cldr, :Unit]}, {:%{}, mm, fields}]}) do
      new_fields =
        Enum.map(fields, fn
          {{:__block__, fm, [:unit]}, value} ->
            {{:__block__, fm, [:name]}, value_to_string(value)}

          {:unit, value} ->
            {:name, value_to_string(value)}

          other ->
            other
        end)

      {:%, m, [{:__aliases__, am, [:Localize, :Unit]}, {:%{}, mm, new_fields}]}
    end

    defp value_to_string({:__block__, m, [atom]}) when is_atom(atom) do
      {:__block__, m, [to_dash_string(atom)]}
    end

    defp value_to_string(other), do: other

    @doc """
    Emits a notice telling the user where to read the rest of the 0.16
    migration story, including the parts that can't be safely auto-rewritten
    (`.unit` field accesses on bare variables, `BB.Safety.report_error/3`
    semantic changes).
    """
    @spec add_release_notice(Igniter.t(), keyword()) :: Igniter.t()
    def add_release_notice(igniter, _opts) do
      Igniter.add_notice(igniter, """
      bb 0.16 introduces two breaking changes. The upgrader has applied the
      mechanical migrations; a few manual follow-ups may still be needed.

      Read documentation/how-to/upgrade-to-0.16.md (or the corresponding
      hexdocs page) for the full migration guide. Key things the upgrader
      cannot safely automate:

        * `value.unit` field accesses now need to be `value.name` and the
          value is a string, not an atom. We don't rewrite bare `.unit`
          accesses because we can't be sure of the receiver's type.

        * `BB.Safety.report_error/3` no longer disarms the robot — it just
          publishes a `[:safety, :error]` event. If you relied on the old
          auto-disarm behaviour, escalate via crashing (raise/exit) so the
          topology supervisor's restart budget can trigger force-disarm.
      """)
    end

    # --- internal helpers ----------------------------------------------------

    defp find_bb_user_modules(igniter) do
      ProjectModule.find_all_matching_modules(igniter, fn _mod, zipper ->
        uses_any?(zipper, @bb_use_macros) or implements_any_behaviour?(zipper, @bb_behaviours)
      end)
    end

    defp find_modules_using(igniter, modules) do
      ProjectModule.find_all_matching_modules(igniter, fn _mod, zipper ->
        uses_any?(zipper, modules)
      end)
    end

    defp uses_any?(zipper, modules) do
      Enum.any?(modules, fn module ->
        match?({:ok, _}, Module.move_to_use(zipper, module))
      end)
    end

    defp implements_any_behaviour?(zipper, modules) do
      Enum.any?(modules, fn module ->
        match?({:ok, _}, Common.move_to(zipper, &behaviour_node?(&1, module)))
      end)
    end

    defp behaviour_node?(%Zipper{node: node}, module) do
      target = module |> Elixir.Module.split() |> Enum.map(&String.to_atom/1)

      case node do
        {:@, _, [{:behaviour, _, [{:__aliases__, _, ^target}]}]} -> true
        _ -> false
      end
    rescue
      _ -> false
    end

    defp has_do_block?(zipper) do
      match?({:ok, _}, Common.move_to_do_block(zipper))
    end
  end
end
