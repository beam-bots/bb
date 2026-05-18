# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Sourceror) do
  defmodule BB.Urdf.Importer do
    @moduledoc """
    Generate BB DSL source code from a parsed URDF document.

    Consumes the intermediate representation produced by `BB.Urdf.Parser` and
    emits an Elixir `defmodule` that `use BB`, with nested `link`/`joint` blocks
    in topology order. The output is formatted source ready to be written into
    a file.

    ## Limitations

    URDF features that don't have a direct BB analogue are skipped with a
    warning rather than failing the import — see `BB.Urdf.Parser` for the
    warnings collected during parsing. The importer adds its own warnings for
    topology issues (cycles, missing parent/child links, multiple roots).
    """

    alias BB.Urdf.Parser

    @cardinal_axes %{
      {1.0, 0.0, 0.0} => [pitch: 90.0],
      {-1.0, 0.0, 0.0} => [pitch: -90.0],
      {0.0, 1.0, 0.0} => [roll: -90.0],
      {0.0, -1.0, 0.0} => [roll: 90.0],
      {0.0, 0.0, 1.0} => [],
      {0.0, 0.0, -1.0} => [pitch: 180.0]
    }

    @doc """
    Render a parsed URDF document to a formatted Elixir source string.

    Returns `{:ok, source, warnings}` on success.
    """
    @spec to_source(Parser.robot(), module) :: {:ok, String.t(), [String.t()]} | {:error, term}
    def to_source(robot, module_name) when is_atom(module_name) do
      with {:ok, ast, warnings} <- to_quoted(robot, module_name) do
        source = Sourceror.to_string(ast, locals_without_parens: locals_without_parens())
        {:ok, source <> "\n", warnings}
      end
    end

    @doc """
    Build the quoted form of the generated module.
    """
    @spec to_quoted(Parser.robot(), module) ::
            {:ok, Macro.t(), [String.t()]} | {:error, term}
    def to_quoted(robot, module_name) when is_atom(module_name) do
      links_by_name = Map.new(robot.links, &{&1.name, &1})
      joints_by_parent = Enum.group_by(robot.joints, & &1.parent)
      child_links = MapSet.new(robot.joints, & &1.child)

      with :ok <- validate_referenced_links(robot.joints, links_by_name),
           {:ok, root} <- roots(robot.links, child_links) do
        topology = render_topology(root, links_by_name, joints_by_parent)
        settings = render_settings(robot.name)

        body =
          block([
            call(:use, [{:__aliases__, [], [:BB]}]),
            settings,
            topology
          ])

        module_alias =
          {:__aliases__, [], Module.split(module_name) |> Enum.map(&String.to_atom/1)}

        ast = {:defmodule, [], [module_alias, [do: body]]}

        {:ok, ast, robot.warnings}
      end
    end

    defp validate_referenced_links(joints, links_by_name) do
      Enum.reduce_while(joints, :ok, fn joint, _acc ->
        cond do
          not Map.has_key?(links_by_name, joint.parent) ->
            {:halt, {:error, {:undefined_link, joint.name, joint.parent}}}

          not Map.has_key?(links_by_name, joint.child) ->
            {:halt, {:error, {:undefined_link, joint.name, joint.child}}}

          true ->
            {:cont, :ok}
        end
      end)
    end

    defp roots(links, child_links) do
      case Enum.reject(links, &MapSet.member?(child_links, &1.name)) do
        [root] ->
          {:ok, root}

        [] ->
          {:error, :no_root_link}

        multiple ->
          names = Enum.map(multiple, & &1.name)
          {:error, {:multiple_root_links, names}}
      end
    end

    defp render_settings(name) do
      call(:settings, [], [call(:name, [atom_name(name)])])
    end

    defp render_topology(root, links_by_name, joints_by_parent) do
      call(:topology, [], [render_link(root, links_by_name, joints_by_parent)])
    end

    defp render_link(link, links_by_name, joints_by_parent) do
      children =
        render_inertial(link.inertial) ++
          render_visual(link.visual) ++
          Enum.map(link.collisions, &render_collision/1) ++
          Enum.flat_map(Map.get(joints_by_parent, link.name, []), fn joint ->
            [render_joint(joint, links_by_name, joints_by_parent)]
          end)

      call(:link, [atom_name(link.name)], children)
    end

    defp render_inertial(nil), do: []

    defp render_inertial(inertial) do
      children =
        maybe(inertial.mass, &call(:mass, [unit(&1, :kilogram)])) ++
          maybe(inertial.inertia, &render_inertia/1) ++
          maybe(inertial.origin, &render_origin/1)

      [call(:inertial, [], children)]
    end

    defp render_inertia(inertia) do
      call(:inertia, [], [
        call(:ixx, [unit(inertia.ixx, :kilogram_square_meter)]),
        call(:iyy, [unit(inertia.iyy, :kilogram_square_meter)]),
        call(:izz, [unit(inertia.izz, :kilogram_square_meter)]),
        call(:ixy, [unit(inertia.ixy, :kilogram_square_meter)]),
        call(:ixz, [unit(inertia.ixz, :kilogram_square_meter)]),
        call(:iyz, [unit(inertia.iyz, :kilogram_square_meter)])
      ])
    end

    defp render_visual(nil), do: []

    defp render_visual(visual) do
      children =
        maybe(visual.origin, &render_origin/1) ++
          maybe(visual.geometry, &render_geometry/1) ++
          maybe(visual.material, &render_material/1)

      [call(:visual, [], children)]
    end

    defp render_collision(collision) do
      children =
        maybe(collision.origin, &render_origin/1) ++
          maybe(collision.name, &call(:name, [atom_name(&1)])) ++
          maybe(collision.geometry, &render_geometry/1)

      call(:collision, [], children)
    end

    defp render_geometry({:box, %{size: {x, y, z}}}) do
      call(:box, [], [
        call(:x, [unit(x, :meter)]),
        call(:y, [unit(y, :meter)]),
        call(:z, [unit(z, :meter)])
      ])
    end

    defp render_geometry({:cylinder, %{radius: r, length: l}}) do
      call(:cylinder, [], [
        call(:radius, [unit(r, :meter)]),
        call(:height, [unit(l, :meter)])
      ])
    end

    defp render_geometry({:sphere, %{radius: r}}) do
      call(:sphere, [], [call(:radius, [unit(r, :meter)])])
    end

    defp render_geometry({:mesh, %{filename: filename, scale: scale}}) do
      children =
        [call(:filename, [filename])] ++
          if scale == 1.0, do: [], else: [call(:scale, [scale])]

      call(:mesh, [], children)
    end

    defp render_material(material) do
      children =
        maybe(material.name, &call(:name, [atom_name(&1)])) ++
          maybe(material.color, &render_color/1) ++
          maybe(material.texture, &render_texture/1)

      call(:material, [], children)
    end

    defp render_color(%{red: r, green: g, blue: b, alpha: a}) do
      call(:color, [], [
        call(:red, [r]),
        call(:green, [g]),
        call(:blue, [b]),
        call(:alpha, [a])
      ])
    end

    defp render_texture(filename) when is_binary(filename) do
      call(:texture, [], [call(:filename, [filename])])
    end

    defp render_joint(joint, links_by_name, joints_by_parent) do
      child_link = Map.fetch!(links_by_name, joint.child)

      body =
        [call(:type, [joint.type])] ++
          maybe(joint.origin, &render_origin/1) ++
          render_axis(joint) ++
          render_limit(joint) ++
          render_dynamics(joint.dynamics) ++
          [render_link(child_link, links_by_name, joints_by_parent)]

      call(:joint, [atom_name(joint.name)], body)
    end

    defp render_axis(%{type: :fixed}), do: []
    defp render_axis(%{axis: nil}), do: []

    defp render_axis(%{axis: axis}) do
      children =
        case Map.get(@cardinal_axes, normalise_axis(axis)) do
          nil ->
            {roll, pitch} = axis_to_euler(axis)

            [
              call(:roll, [unit(roll, :degree)]),
              call(:pitch, [unit(pitch, :degree)])
            ]

          angles ->
            Enum.map(angles, fn {key, deg} -> call(key, [unit(deg, :degree)]) end)
        end

      [empty_block_call(:axis, [], children)]
    end

    defp render_limit(%{type: :fixed}), do: []
    defp render_limit(%{limit: nil}), do: []

    defp render_limit(%{type: :continuous, limit: limit}) do
      children =
        maybe(limit.effort, &call(:effort, [unit(&1, :newton_meter)])) ++
          maybe(limit.velocity, &call(:velocity, [unit(&1, :radian_per_second)]))

      if children == [], do: [], else: [call(:limit, [], children)]
    end

    defp render_limit(%{type: type, limit: limit}) do
      {position_unit, velocity_unit, effort_unit} = limit_units_for_type(type)

      children =
        maybe(limit.lower, &call(:lower, [unit(&1, position_unit)])) ++
          maybe(limit.upper, &call(:upper, [unit(&1, position_unit)])) ++
          maybe(limit.effort, &call(:effort, [unit(&1, effort_unit)])) ++
          maybe(limit.velocity, &call(:velocity, [unit(&1, velocity_unit)]))

      if children == [], do: [], else: [call(:limit, [], children)]
    end

    defp limit_units_for_type(:prismatic), do: {:meter, :meter_per_second, :newton}
    defp limit_units_for_type(_), do: {:radian, :radian_per_second, :newton_meter}

    defp render_dynamics(nil), do: []

    defp render_dynamics(%{damping: nil, friction: nil}), do: []

    defp render_dynamics(dynamics) do
      children =
        maybe(dynamics.damping, &call(:damping, [unit(&1, :newton_meter_second_per_radian)])) ++
          maybe(dynamics.friction, &call(:friction, [unit(&1, :newton_meter)]))

      [call(:dynamics, [], children)]
    end

    defp render_origin(%{xyz: {x, y, z}, rpy: {roll, pitch, yaw}}) do
      case drop_zero([
             {:x, x, :meter},
             {:y, y, :meter},
             {:z, z, :meter},
             {:roll, roll, :radian},
             {:pitch, pitch, :radian},
             {:yaw, yaw, :radian}
           ]) do
        [] -> []
        children -> [call(:origin, [], children)]
      end
    end

    defp drop_zero(fields) do
      fields
      |> Enum.reject(fn {_k, v, _u} -> v == 0.0 end)
      |> Enum.map(fn {k, v, u} -> call(k, [unit(v, u)]) end)
    end

    defp normalise_axis({x, y, z}) do
      {round_axis(x), round_axis(y), round_axis(z)}
    end

    defp round_axis(v) when v > 0.999 and v < 1.001, do: 1.0
    defp round_axis(v) when v < -0.999 and v > -1.001, do: -1.0
    defp round_axis(v) when v > -0.001 and v < 0.001, do: 0.0
    defp round_axis(v), do: v

    # For non-cardinal axis vectors, compute roll/pitch that rotate Z to the
    # target. Yaw is redundant (rotation about Z) so we leave it at zero.
    defp axis_to_euler({x, y, z}) do
      pitch = :math.atan2(x, z)
      roll = :math.atan2(-y, :math.sqrt(x * x + z * z))
      {rad_to_deg(roll), rad_to_deg(pitch)}
    end

    defp rad_to_deg(r), do: r * 180.0 / :math.pi()

    defp atom_name(name) when is_atom(name), do: name

    defp atom_name(name) when is_binary(name) do
      name
      |> String.replace(~r/[^A-Za-z0-9_]/, "_")
      |> then(fn
        <<digit, _::binary>> = s when digit in ?0..?9 -> "_" <> s
        s -> s
      end)
      |> String.to_atom()
    end

    defp unit(value, unit_name) when is_float(value) and is_atom(unit_name) do
      text = "#{format_float(value)} #{unit_name}"
      {:sigil_u, [delimiter: "("], [{:<<>>, [], [text]}, []]}
    end

    defp format_float(v) when is_float(v) do
      v
      |> :erlang.float_to_binary(decimals: 6)
      |> String.trim_trailing("0")
      |> String.trim_trailing(".")
      |> case do
        "" -> "0"
        "-" -> "0"
        other -> other
      end
    end

    defp call(name, args, []), do: {name, [], args}

    defp call(name, args, children) when is_list(children),
      do: {name, [], args ++ [[do: block(children)]]}

    defp call(name, args), do: {name, [], args}

    defp empty_block_call(name, args, []),
      do: {name, [], args ++ [[do: {:__block__, [], []}]]}

    defp empty_block_call(name, args, children),
      do: call(name, args, children)

    defp block([single]), do: single
    defp block(children), do: {:__block__, [], children}

    defp maybe(nil, _fun), do: []

    defp maybe(value, fun) do
      case fun.(value) do
        list when is_list(list) -> list
        ast -> [ast]
      end
    end

    # Mirrors the BB DSL entries in `.formatter.exs` for the entities emitted
    # by this module, so generated source uses idiomatic DSL formatting without
    # requiring the consumer to run `mix format` first.
    defp locals_without_parens do
      [
        alpha: 1,
        axis: 0,
        axis: 1,
        blue: 1,
        box: 0,
        box: 1,
        collision: 0,
        collision: 1,
        color: 0,
        color: 1,
        cylinder: 0,
        cylinder: 1,
        damping: 1,
        dynamics: 0,
        dynamics: 1,
        effort: 1,
        filename: 1,
        friction: 1,
        green: 1,
        height: 1,
        inertia: 0,
        inertia: 1,
        inertial: 0,
        inertial: 1,
        ixx: 1,
        ixy: 1,
        ixz: 1,
        iyy: 1,
        iyz: 1,
        izz: 1,
        joint: 1,
        joint: 2,
        length: 1,
        limit: 0,
        limit: 1,
        link: 1,
        link: 2,
        lower: 1,
        mass: 1,
        material: 0,
        material: 1,
        mesh: 0,
        mesh: 1,
        name: 1,
        origin: 0,
        origin: 1,
        pitch: 1,
        radius: 1,
        red: 1,
        roll: 1,
        scale: 1,
        settings: 1,
        sphere: 0,
        sphere: 1,
        texture: 0,
        texture: 1,
        topology: 1,
        type: 1,
        upper: 1,
        velocity: 1,
        visual: 0,
        visual: 1,
        x: 1,
        y: 1,
        yaw: 1,
        z: 1
      ]
    end
  end
end
