# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Axis do
  @moduledoc """
  Joint axis orientation specified as Euler angles.

  The axis defines the direction of rotation (for revolute joints) or
  translation (for prismatic joints). By default, the axis points along
  the Z direction. Use roll, pitch, and yaw to rotate it to the desired
  orientation.

  ## Examples

      # Default Z-axis (no rotation needed)
      axis do
      end

      # Y-axis (pitch by 90°)
      axis do
        pitch(~u(90 degree))
      end

      # X-axis (pitch by 90°, then roll by 90°)
      axis do
        pitch(~u(90 degree))
        roll(~u(90 degree))
      end
  """
  import BB.Unit

  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            roll: ~u(0 degree),
            pitch: ~u(0 degree),
            yaw: ~u(0 degree)

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          roll: Cldr.Unit.t(),
          pitch: Cldr.Unit.t(),
          yaw: Cldr.Unit.t()
        }
end
