# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Robot.Link do
  @moduledoc """
  An optimised link representation with all units converted to SI floats.

  Links are connected to their parent via `parent_joint` (nil for the root link)
  and to children via `child_joints` (list of joint names).
  """

  defstruct [
    :name,
    :parent_joint,
    :child_joints,
    :mass,
    :center_of_mass,
    :inertia,
    :visual,
    :collisions,
    :sensors
  ]

  @type t :: %__MODULE__{
          name: atom(),
          parent_joint: atom() | nil,
          child_joints: [atom()],
          mass: float() | nil,
          center_of_mass: position() | nil,
          inertia: inertia() | nil,
          visual: visual() | nil,
          collisions: [collision()],
          sensors: [atom()]
        }

  @typedoc "Position as {x, y, z} in meters"
  @type position :: {float(), float(), float()}

  @typedoc "Orientation as {roll, pitch, yaw} in radians"
  @type orientation :: {float(), float(), float()}

  @typedoc "Inertia tensor components in kg·m²"
  @type inertia :: %{
          ixx: float(),
          iyy: float(),
          izz: float(),
          ixy: float(),
          ixz: float(),
          iyz: float()
        }

  @typedoc "Visual geometry information"
  @type visual :: %{
          origin: {position(), orientation()} | nil,
          geometry: geometry() | nil,
          material: material() | nil
        }

  @typedoc "Collision geometry information"
  @type collision :: %{
          name: atom() | nil,
          origin: {position(), orientation()} | nil,
          geometry: geometry() | nil
        }

  @typedoc "Geometry specification"
  @type geometry ::
          {:box, %{x: float(), y: float(), z: float()}}
          | {:cylinder, %{radius: float(), height: float()}}
          | {:sphere, %{radius: float()}}
          | {:mesh, %{filename: String.t(), scale: float()}}

  @typedoc "Material specification"
  @type material :: %{
          name: atom(),
          color: color() | nil,
          texture: String.t() | nil
        }

  @typedoc "RGBA color (values 0-1)"
  @type color :: %{red: float(), green: float(), blue: float(), alpha: float()}
end
