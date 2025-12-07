# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.JointState do
  @moduledoc """
  State of a set of joints.

  ## Fields

  - `names` - List of joint names as atoms
  - `positions` - Joint positions in radians (revolute) or metres (prismatic)
  - `velocities` - Joint velocities in rad/s or m/s
  - `efforts` - Joint efforts in Nm or N

  All lists must have the same length. Missing values can be represented
  as empty lists.

  ## Examples

      alias BB.Message.Sensor.JointState

      {:ok, msg} = JointState.new(:arm,
        names: [:joint1, :joint2],
        positions: [0.0, 1.57],
        velocities: [0.1, 0.0],
        efforts: [0.5, 0.2]
      )
  """

  defstruct [:names, :positions, :velocities, :efforts]

  use BB.Message,
    schema: [
      names: [type: {:list, :atom}, required: true, doc: "Joint names"],
      positions: [type: {:list, :float}, default: [], doc: "Joint positions in radians or metres"],
      velocities: [type: {:list, :float}, default: [], doc: "Joint velocities in rad/s or m/s"],
      efforts: [type: {:list, :float}, default: [], doc: "Joint efforts in Nm or N"]
    ]

  @type t :: %__MODULE__{
          names: [atom()],
          positions: [float()],
          velocities: [float()],
          efforts: [float()]
        }
end
