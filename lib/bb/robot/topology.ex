# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Topology do
  @moduledoc """
  Pre-computed topology metadata for efficient traversal and kinematic operations.

  This struct contains ordering information that allows:
  - Forward kinematics to process joints in the correct order
  - Path lookup between any two nodes
  - Depth information for tree operations

  Paths are root-relative and interleave links and joints, starting and ending
  with the node they name — `[:base, :shoulder, :upper_arm, :elbow, :forearm]`.

  Lookups here have no robot name to attribute their errors to. `BB.Robot`
  delegates to them and fills the name in.
  """

  alias BB.Error.Kinematics.NotAnAncestor
  alias BB.Error.Kinematics.UnknownLink

  defstruct [
    :link_order,
    :joint_order,
    :paths,
    :depth
  ]

  @type t :: %__MODULE__{
          link_order: [atom()],
          joint_order: [atom()],
          paths: %{atom() => [atom()]},
          depth: %{atom() => non_neg_integer()}
        }

  @doc """
  Get the depth of a node in the tree.

  The root link has depth 0. Each joint/link pair adds 1 to the depth.
  """
  @spec depth_of(t(), atom()) :: {:ok, non_neg_integer()} | {:error, UnknownLink.t()}
  def depth_of(%__MODULE__{depth: depth}, name) do
    case Map.fetch(depth, name) do
      {:ok, value} -> {:ok, value}
      :error -> {:error, UnknownLink.exception(link: name)}
    end
  end

  @doc """
  Get the path from root to a node.

  Returns a list of link/joint names from the root to the given node.
  """
  @spec path_to(t(), atom()) :: {:ok, [atom()]} | {:error, UnknownLink.t()}
  def path_to(%__MODULE__{paths: paths}, name) do
    case Map.fetch(paths, name) do
      {:ok, path} -> {:ok, path}
      :error -> {:error, UnknownLink.exception(link: name)}
    end
  end

  @doc """
  Get the path from a source node down to a target node.

  Restricted to the case where `source` is an ancestor of `target`, which makes
  this a prefix drop on the precomputed paths. The result starts at `source` and
  ends at `target`; a source equal to the target gives `{:ok, [source]}`.

  Fails with `BB.Error.Kinematics.NotAnAncestor` when the source sits somewhere
  other than above the target, carrying their nearest common ancestor. That
  ancestor always exists: `BB.Dsl.TopologyTransformer` rejects any topology
  without exactly one root link and the DSL's nesting makes cycles impossible,
  so the topology is a single tree in which any two nodes share at least the
  root.
  """
  @spec path_between(t(), atom(), atom()) ::
          {:ok, [atom()]} | {:error, UnknownLink.t() | NotAnAncestor.t()}
  def path_between(%__MODULE__{paths: paths}, source, target) do
    with {:ok, source_path} <- fetch_path(paths, source, :source),
         {:ok, target_path} <- fetch_path(paths, target, :target) do
      descend(source_path, target_path, source, target)
    end
  end

  @doc """
  Get all leaf links (links with no child joints).
  """
  @spec leaf_links(t(), BB.Robot.t()) :: [atom()]
  def leaf_links(%__MODULE__{link_order: link_order}, robot) do
    Enum.filter(link_order, fn link_name ->
      match?({:ok, %BB.Robot.Link{child_joints: []}}, BB.Robot.get_link(robot, link_name))
    end)
  end

  @doc """
  Get the maximum depth of the kinematic tree.
  """
  @spec max_depth(t()) :: non_neg_integer()
  def max_depth(%__MODULE__{depth: depth}) do
    depth
    |> Map.values()
    |> Enum.max(fn -> 0 end)
  end

  defp fetch_path(paths, name, role) do
    case Map.fetch(paths, name) do
      {:ok, path} -> {:ok, path}
      :error -> {:error, UnknownLink.exception(link: name, role: role)}
    end
  end

  defp descend(source_path, target_path, source, target) do
    if List.starts_with?(target_path, source_path) do
      {:ok, Enum.drop(target_path, length(source_path) - 1)}
    else
      {:error,
       NotAnAncestor.exception(
         source_link: source,
         target_link: target,
         common_ancestor: nearest_common_ancestor(source_path, target_path)
       )}
    end
  end

  # Paths alternate link, joint, link, …, so every even index is a link and the
  # deepest common link is the last of them. Both paths start at the same root,
  # so there is always at least one.
  defp nearest_common_ancestor(source_path, target_path) do
    source_path
    |> Enum.zip(target_path)
    |> Enum.take_while(fn {source, target} -> source == target end)
    |> Enum.map(&elem(&1, 0))
    |> Enum.take_every(2)
    |> List.last()
  end
end
