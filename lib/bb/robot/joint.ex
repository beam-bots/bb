# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Joint do
  @moduledoc """
  An optimised joint representation with all units converted to SI floats.

  Joints connect a parent link to a child link and define the kinematic
  relationship between them.
  """

  defstruct [
    :name,
    :type,
    :parent_link,
    :child_link,
    :origin,
    :axis,
    :limits,
    :dynamics,
    :sensors,
    :actuators
  ]

  @type joint_type :: :revolute | :continuous | :prismatic | :fixed | :floating | :planar

  @type t :: %__MODULE__{
          name: atom(),
          type: joint_type(),
          parent_link: atom(),
          child_link: atom(),
          origin: origin() | nil,
          axis: axis() | nil,
          limits: limits() | nil,
          dynamics: dynamics() | nil,
          sensors: [atom()],
          actuators: [atom()]
        }

  @typedoc """
  Joint origin transform from parent to child frame.

  - `position`: {x, y, z} translation in meters
  - `orientation`: {roll, pitch, yaw} rotation in radians (XYZ Euler angles)
  """
  @type origin :: %{
          position: {float(), float(), float()},
          orientation: {float(), float(), float()}
        }

  @typedoc """
  Joint axis of rotation/translation as a normalised unit vector {x, y, z}.
  """
  @type axis :: {float(), float(), float()}

  @typedoc """
  Joint limits.

  For revolute/continuous joints:
  - `lower`/`upper`: angle limits in radians
  - `velocity`: max angular velocity in rad/s
  - `effort`: max torque in N·m
  - `acceleration`: max angular acceleration in rad/s² (optional)

  For prismatic joints:
  - `lower`/`upper`: position limits in meters
  - `velocity`: max linear velocity in m/s
  - `effort`: max effort in N·m (as defined in DSL)
  - `acceleration`: max linear acceleration in m/s² (optional)
  """
  @type limits :: %{
          lower: float() | nil,
          upper: float() | nil,
          velocity: float(),
          effort: float(),
          acceleration: float() | nil
        }

  @typedoc """
  Joint dynamics parameters.

  - `damping`: viscous damping coefficient
    - For revolute: N·m·s/rad
    - For prismatic: N·s/m
  - `friction`: Coulomb friction
    - For revolute: N·m
    - For prismatic: N
  """
  @type dynamics :: %{
          damping: float() | nil,
          friction: float() | nil
        }

  @doc """
  Check if this joint is rotational (revolute or continuous).
  """
  @spec rotational?(t()) :: boolean()
  def rotational?(%__MODULE__{type: type}) when type in [:revolute, :continuous], do: true
  def rotational?(%__MODULE__{}), do: false

  @doc """
  Check if this joint is linear (prismatic).
  """
  @spec linear?(t()) :: boolean()
  def linear?(%__MODULE__{type: :prismatic}), do: true
  def linear?(%__MODULE__{}), do: false

  @doc """
  Check if this joint has any degrees of freedom.
  """
  @spec movable?(t()) :: boolean()
  def movable?(%__MODULE__{type: :fixed}), do: false
  def movable?(%__MODULE__{}), do: true

  @doc """
  How many degrees of freedom this joint has.

  This is the number of Jacobian columns the joint contributes, and the size of
  its configuration — see `BB.Robot.State` for the shape each type carries.

  ## Examples

      iex> BB.Robot.Joint.dof(%BB.Robot.Joint{type: :revolute})
      1

      iex> BB.Robot.Joint.dof(%BB.Robot.Joint{type: :floating})
      6
  """
  @spec dof(t()) :: 0 | 1 | 3 | 6
  def dof(%__MODULE__{type: :fixed}), do: 0
  def dof(%__MODULE__{type: :planar}), do: 3
  def dof(%__MODULE__{type: :floating}), do: 6
  def dof(%__MODULE__{}), do: 1
end
