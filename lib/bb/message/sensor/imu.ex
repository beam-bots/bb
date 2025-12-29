# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.Imu do
  @moduledoc """
  Inertial Measurement Unit data.

  ## Fields

  - `orientation` - Orientation as `BB.Quaternion.t()`
  - `angular_velocity` - Angular velocity as `BB.Vec3.t()` in rad/s
  - `linear_acceleration` - Linear acceleration as `BB.Vec3.t()` in m/s²

  ## Examples

      alias BB.Message.Sensor.Imu
      alias BB.{Vec3, Quaternion}

      {:ok, msg} = Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
      )
  """

  import BB.Message.Option

  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  defstruct [:orientation, :angular_velocity, :linear_acceleration]

  use BB.Message,
    schema: [
      orientation: [type: quaternion_type(), required: true, doc: "Orientation as quaternion"],
      angular_velocity: [type: vec3_type(), required: true, doc: "Angular velocity in rad/s"],
      linear_acceleration: [
        type: vec3_type(),
        required: true,
        doc: "Linear acceleration in m/s²"
      ]
    ]

  @type t :: %__MODULE__{
          orientation: Quaternion.t(),
          angular_velocity: Vec3.t(),
          linear_acceleration: Vec3.t()
        }
end
