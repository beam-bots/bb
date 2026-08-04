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

  alias BB.Error.Kinematics.NoParentJoint
  alias BB.Error.Kinematics.NotAnAncestor
  alias BB.Error.Kinematics.UnknownActuator
  alias BB.Error.Kinematics.UnknownJoint
  alias BB.Error.Kinematics.UnknownLink
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

  @type param_location ::
          {:joint, atom(), [atom()]}
          | {:actuator, atom(), [atom()]}
          | {:sensor, atom(), [atom()]}

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

  @type transmission :: %{
          reduction: float() | nil,
          offset: float() | nil,
          reversed?: boolean() | nil
        }

  @type sensor_info :: %{
          name: atom(),
          attached_to: {:link, atom()} | {:joint, atom()} | :robot,
          transmission: transmission() | nil
        }

  @type actuator_info :: %{
          name: atom(),
          joint: atom(),
          transmission: transmission() | nil
        }

  @doc """
  Get the link at the root of the kinematic tree.

  Returns a bare atom rather than a result tuple: unlike every other lookup here
  it cannot fail, because `BB.Dsl.TopologyTransformer` guarantees exactly one
  root link exists.

  Useful when a caller genuinely wants a whole-tree chain and has to say so —
  `BB.Motion` requires `:source_link` with no default, precisely so that
  root-to-target is recorded as a decision rather than assumed.

      BB.Motion.move_to(robot, :gripper, target,
        source_link: BB.Robot.root_link(robot),
        solver: BB.IK.DLS
      )
  """
  @spec root_link(t()) :: atom()
  def root_link(%__MODULE__{root_link: root_link}), do: root_link

  @doc """
  Get a link by name.
  """
  @spec get_link(t(), atom()) :: {:ok, Link.t()} | {:error, UnknownLink.t()}
  def get_link(%__MODULE__{links: links, name: robot_name}, name) do
    case Map.fetch(links, name) do
      {:ok, link} -> {:ok, link}
      :error -> {:error, UnknownLink.exception(link: name, robot: robot_name)}
    end
  end

  @doc """
  Get a joint by name.
  """
  @spec get_joint(t(), atom()) :: {:ok, Joint.t()} | {:error, UnknownJoint.t()}
  def get_joint(%__MODULE__{joints: joints, name: robot_name}, name) do
    case Map.fetch(joints, name) do
      {:ok, joint} -> {:ok, joint}
      :error -> {:error, UnknownJoint.exception(joint: name, robot: robot_name)}
    end
  end

  @doc """
  Get the parent joint of a link.

  The root link has no parent joint, which is reported as
  `{:error, %BB.Error.Kinematics.NoParentJoint{}}` — a distinct type from
  `UnknownLink` so a caller walking up the tree can match on it as a termination
  signal rather than being told a valid root link doesn't exist.
  """
  @spec parent_joint(t(), atom()) ::
          {:ok, Joint.t()} | {:error, UnknownLink.t() | UnknownJoint.t() | NoParentJoint.t()}
  def parent_joint(%__MODULE__{} = robot, link_name) do
    case get_link(robot, link_name) do
      {:ok, %Link{parent_joint: nil}} -> {:error, NoParentJoint.exception(link: link_name)}
      {:ok, %Link{parent_joint: joint_name}} -> get_joint(robot, joint_name)
      {:error, reason} -> {:error, reason}
    end
  end

  @doc """
  Get the child joints of a link.

  A link with no children returns `{:ok, []}`, which is distinct from naming a
  link that doesn't exist.
  """
  @spec child_joints(t(), atom()) :: {:ok, [Joint.t()]} | {:error, UnknownLink.t()}
  def child_joints(%__MODULE__{joints: joints} = robot, link_name) do
    case get_link(robot, link_name) do
      {:ok, %Link{child_joints: joint_names}} ->
        {:ok, Enum.map(joint_names, &Map.fetch!(joints, &1))}

      {:error, reason} ->
        {:error, reason}
    end
  end

  @doc """
  Get the path from root to a given link or joint.

  Equivalent to `path_between/3` from the root link, and delegates to
  `BB.Robot.Topology.path_to/2`.
  """
  @spec path_to(t(), atom()) :: {:ok, [atom()]} | {:error, UnknownLink.t()}
  def path_to(%__MODULE__{topology: topology} = robot, name) do
    topology |> Topology.path_to(name) |> attribute_to(robot)
  end

  @doc """
  Get the path from a source link down to a target link.

  Restricted to the case where `source_link` is an ancestor of `target_link`,
  which is a prefix drop on the precomputed root-relative paths. The result
  starts at `source_link` and ends at `target_link`, interleaving the joints and
  links between them, so `path_to/2` is the special case of a source at the root.

  A source that isn't above the target reports
  `BB.Error.Kinematics.NotAnAncestor`, carrying the nearest common ancestor so
  the message names the link the caller should have passed.

      BB.Robot.path_between(robot, :chassis, :sensor_head)
      #=> {:ok, [:chassis, :mast, :sensor_head]}
  """
  @spec path_between(t(), atom(), atom()) ::
          {:ok, [atom()]} | {:error, UnknownLink.t() | NotAnAncestor.t()}
  def path_between(%__MODULE__{topology: topology} = robot, source_link, target_link) do
    topology |> Topology.path_between(source_link, target_link) |> attribute_to(robot)
  end

  @doc """
  Get the full path from root to an actuator.

  Actuator names are unique per robot, so the path is derivable: an actuator
  always hangs off a joint, and the joint's own path gives the links and joints
  above it. The result matches the `:path` the framework injects into the
  actuator's `:bb` option, and therefore the topic its commands are published
  to — `[:actuator | actuator_path(robot, name)]`.

      BB.Robot.actuator_path(robot, :pan_servo)
      #=> {:ok, [:base, :pan, :pan_servo]}
  """
  @spec actuator_path(t(), atom()) :: {:ok, [atom()]} | {:error, UnknownActuator.t()}
  def actuator_path(%__MODULE__{actuators: actuators, name: robot_name} = robot, name) do
    with {:ok, %{joint: joint_name}} <- Map.fetch(actuators, name),
         {:ok, joint_path} <- path_to(robot, joint_name) do
      {:ok, joint_path ++ [name]}
    else
      _ -> {:error, UnknownActuator.exception(actuator: name, robot: robot_name)}
    end
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

  # `BB.Robot.Topology` has no robot name to put in its errors, so name them here
  # rather than leaving the diagnostic weaker than the error type allows.
  defp attribute_to({:error, %UnknownLink{} = error}, %__MODULE__{name: name}) do
    {:error, %{error | robot: name}}
  end

  defp attribute_to(result, %__MODULE__{}), do: result
end
