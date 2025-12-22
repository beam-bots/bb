# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot do
  @moduledoc """
  An optimised robot representation for kinematic computations.

  This struct is built from the Spark DSL at compile-time and contains:
  - All physical values converted to SI base units (floats)
  - Flat maps for O(1) lookup of links, joints, sensors, and actuators by name
  - Pre-computed topology metadata for efficient traversal
  - Bidirectional parent/child references

  ## Structure

  The robot is organised as flat maps indexed by name:

  - `links` - all links in the robot, keyed by atom name
  - `joints` - all joints in the robot, keyed by atom name
  - `sensors` - all sensors (at any level), keyed by atom name
  - `actuators` - all actuators, keyed by atom name

  ## Unit Conventions

  All physical quantities are stored as native floats in SI base units:

  - Length: meters
  - Angle: radians
  - Mass: kilograms
  - Moment of inertia: kg·m²
  - Force: newtons
  - Torque: newton-meters
  - Linear velocity: m/s
  - Angular velocity: rad/s
  """

  alias BB.Robot.{Joint, Link, Topology}

  defstruct [
    :name,
    :root_link,
    :links,
    :joints,
    :sensors,
    :actuators,
    :topology,
    param_subscriptions: %{}
  ]

  @type param_location :: {:joint, atom(), [atom()]}

  @type t :: %__MODULE__{
          name: atom(),
          root_link: atom(),
          links: %{atom() => Link.t()},
          joints: %{atom() => Joint.t()},
          sensors: %{atom() => sensor_info()},
          actuators: %{atom() => actuator_info()},
          topology: Topology.t(),
          param_subscriptions: %{[atom()] => [param_location()]}
        }

  @type sensor_info :: %{
          name: atom(),
          attached_to: {:link, atom()} | {:joint, atom()} | :robot
        }

  @type actuator_info :: %{
          name: atom(),
          joint: atom()
        }

  @doc """
  Get a link by name.
  """
  @spec get_link(t(), atom()) :: Link.t() | nil
  def get_link(%__MODULE__{links: links}, name) do
    Map.get(links, name)
  end

  @doc """
  Get a joint by name.
  """
  @spec get_joint(t(), atom()) :: Joint.t() | nil
  def get_joint(%__MODULE__{joints: joints}, name) do
    Map.get(joints, name)
  end

  @doc """
  Get the parent joint of a link (nil for root link).
  """
  @spec parent_joint(t(), atom()) :: Joint.t() | nil
  def parent_joint(%__MODULE__{} = robot, link_name) do
    case get_link(robot, link_name) do
      %Link{parent_joint: nil} -> nil
      %Link{parent_joint: joint_name} -> get_joint(robot, joint_name)
      nil -> nil
    end
  end

  @doc """
  Get the child joints of a link.
  """
  @spec child_joints(t(), atom()) :: [Joint.t()]
  def child_joints(%__MODULE__{} = robot, link_name) do
    case get_link(robot, link_name) do
      %Link{child_joints: joint_names} ->
        Enum.map(joint_names, &get_joint(robot, &1))

      nil ->
        []
    end
  end

  @doc """
  Get the path from root to a given link or joint.
  """
  @spec path_to(t(), atom()) :: [atom()] | nil
  def path_to(%__MODULE__{topology: topology}, name) do
    Map.get(topology.paths, name)
  end

  @doc """
  Get all links in topological order (root first).
  """
  @spec links_in_order(t()) :: [Link.t()]
  def links_in_order(%__MODULE__{topology: topology, links: links}) do
    Enum.map(topology.link_order, &Map.fetch!(links, &1))
  end

  @doc """
  Get all joints in traversal order.
  """
  @spec joints_in_order(t()) :: [Joint.t()]
  def joints_in_order(%__MODULE__{topology: topology, joints: joints}) do
    Enum.map(topology.joint_order, &Map.fetch!(joints, &1))
  end
end
