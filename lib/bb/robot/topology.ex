# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Topology do
  @moduledoc """
  Pre-computed topology metadata for efficient traversal and kinematic operations.

  This struct contains ordering information that allows:
  - Forward kinematics to process joints in the correct order
  - Path lookup from root to any node
  - Depth information for tree operations
  """

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
  @spec depth_of(t(), atom()) :: non_neg_integer() | nil
  def depth_of(%__MODULE__{depth: depth}, name) do
    Map.get(depth, name)
  end

  @doc """
  Get the path from root to a node.

  Returns a list of link/joint names from the root to the given node.
  """
  @spec path_to(t(), atom()) :: [atom()] | nil
  def path_to(%__MODULE__{paths: paths}, name) do
    Map.get(paths, name)
  end

  @doc """
  Get all leaf links (links with no child joints).
  """
  @spec leaf_links(t(), BB.Robot.t()) :: [atom()]
  def leaf_links(%__MODULE__{link_order: link_order}, robot) do
    Enum.filter(link_order, fn link_name ->
      case BB.Robot.get_link(robot, link_name) do
        %BB.Robot.Link{child_joints: []} -> true
        _ -> false
      end
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
end
