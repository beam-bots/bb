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
      with {:ok, prepared} <- prepare(robot) do
        topology =
          render_topology(prepared.root, prepared.links_by_name, prepared.joints_by_parent)

        settings = render_settings(prepared.name)

        body =
          block([
            call(:use, [{:__aliases__, [], [:BB]}]),
            settings,
            topology
          ])

        module_alias =
          {:__aliases__, [], Module.split(module_name) |> Enum.map(&String.to_atom/1)}

        ast = {:defmodule, [], [module_alias, [do: body]]}

        {:ok, ast, prepared.warnings}
      end
    end

    @doc """
    Build the quoted form of just the `topology do ... end` block.

    Used when merging an imported URDF into an existing BB module — only the
    topology gets replaced, leaving `settings`, sensors, controllers, commands
    and other hand-written content alone.
    """
    @spec to_topology_quoted(Parser.robot()) ::
            {:ok, Macro.t(), [String.t()]} | {:error, term}
    def to_topology_quoted(robot) do
      with {:ok, prepared} <- prepare(robot) do
        topology =
          render_topology(prepared.root, prepared.links_by_name, prepared.joints_by_parent)

        {:ok, topology, prepared.warnings}
      end
    end

    defp prepare(robot) do
      robot =
        robot
        |> drop_world_anchor_joints()
        |> dedupe_joint_names()
        |> dedupe_material_names()

      links_by_name = Map.new(robot.links, &{&1.name, &1})
      joints_by_parent = Enum.group_by(robot.joints, & &1.parent)
      child_links = MapSet.new(robot.joints, & &1.child)

      with :ok <- validate_referenced_links(robot.joints, links_by_name),
           {:ok, root} <- roots(robot.links, child_links) do
        {:ok,
         %{
           root: root,
           links_by_name: links_by_name,
           joints_by_parent: joints_by_parent,
           name: robot.name,
           warnings: robot.warnings
         }}
      end
    end

    # URDF commonly anchors a robot to the world with a fixed joint whose
    # parent is a synthetic link (`world`, `map`, etc.) that's never defined
    # in the file. BB has no concept of a world frame — the topology root is
    # the robot. Drop joints whose parent link isn't defined; their child
    # then becomes an unparented link, which the root-finder picks up
    # naturally.
    defp drop_world_anchor_joints(robot) do
      link_names = MapSet.new(robot.links, & &1.name)

      {joints, dropped} =
        Enum.split_with(robot.joints, &MapSet.member?(link_names, &1.parent))

      warnings =
        Enum.map(dropped, fn joint ->
          "dropped joint #{inspect(joint.name)}: parent link #{inspect(joint.parent)} is not defined (URDF world anchor?)"
        end)

      %{robot | joints: joints, warnings: robot.warnings ++ warnings}
    end

    # URDF has separate namespaces for link and joint names; BB requires
    # global uniqueness across both. Rename any joint whose name collides
    # with a link (or with another joint) and rewrite `<mimic>` source
    # references to match.
    defp dedupe_joint_names(robot) do
      link_names = MapSet.new(robot.links, & &1.name)
      {joints, _used, renames} = rename_colliding_joints(robot.joints, link_names)
      joints = rewrite_mimic_sources(joints, renames)
      %{robot | joints: joints}
    end

    defp rename_colliding_joints(joints, link_names) do
      Enum.reduce(joints, {[], link_names, %{}}, fn joint, {acc, used, renames} ->
        if MapSet.member?(used, joint.name) do
          new_name = unique_name(joint.name <> "_joint", used)

          {[%{joint | name: new_name} | acc], MapSet.put(used, new_name),
           Map.put(renames, joint.name, new_name)}
        else
          {[joint | acc], MapSet.put(used, joint.name), renames}
        end
      end)
      |> then(fn {joints, used, renames} -> {Enum.reverse(joints), used, renames} end)
    end

    defp unique_name(base, used) do
      if MapSet.member?(used, base) do
        Enum.find_value(Stream.iterate(2, &(&1 + 1)), &numbered_candidate(base, &1, used))
      else
        base
      end
    end

    defp numbered_candidate(base, n, used) do
      candidate = "#{base}_#{n}"
      if MapSet.member?(used, candidate), do: nil, else: candidate
    end

    defp rewrite_mimic_sources(joints, renames) when map_size(renames) == 0, do: joints

    defp rewrite_mimic_sources(joints, renames) do
      Enum.map(joints, fn
        %{mimic: %{joint: source} = mimic} = joint ->
          %{joint | mimic: %{mimic | joint: Map.get(renames, source, source)}}

        joint ->
          joint
      end)
    end

    # URDF lets many visuals reference a single named material; BB's DSL
    # requires globally-unique entity names. Keep the URDF name on the first
    # visual that uses each material and strip it on later occurrences (BB
    # auto-generates a unique identifier when `name` is absent).
    defp dedupe_material_names(robot) do
      {links, _seen} = Enum.map_reduce(robot.links, MapSet.new(), &dedupe_link_material/2)
      %{robot | links: links}
    end

    defp dedupe_link_material(%{visual: %{material: %{name: name}}} = link, seen)
         when is_binary(name) do
      if MapSet.member?(seen, name) do
        {strip_material_name(link), seen}
      else
        {link, MapSet.put(seen, name)}
      end
    end

    defp dedupe_link_material(link, seen), do: {link, seen}

    defp strip_material_name(link) do
      visual = %{link.visual | material: %{link.visual.material | name: nil}}
      %{link | visual: visual}
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
      call(:mesh, [], [
        call(:filename, [filename]),
        call(:scale, [scale * 1.0])
      ])
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
          render_mimic(joint) ++
          [render_link(child_link, links_by_name, joints_by_parent)]

      call(:joint, [atom_name(joint.name)], body)
    end

    defp render_mimic(%{mimic: nil}), do: []

    defp render_mimic(%{name: joint_name, mimic: mimic}) do
      sensor_name = String.to_atom("#{joint_name}_mimic")

      opts =
        [source: atom_name(mimic.joint)]
        |> append_if(mimic.multiplier != 1.0, multiplier: mimic.multiplier)
        |> append_if(mimic.offset != 0.0, offset: mimic.offset)

      mimic_module = {:__aliases__, [], [:BB, :Sensor, :Mimic]}
      child_spec = {mimic_module, opts}

      [call(:sensor, [sensor_name, child_spec])]
    end

    defp append_if(list, false, _kv), do: list
    defp append_if(list, true, kv), do: list ++ kv

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
        sensor: 2,
        sensor: 3,
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
