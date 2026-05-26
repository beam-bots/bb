# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.Imu do
  @moduledoc """
  Inertial Measurement Unit data.

  ## Fields

  - `orientation` - Orientation as `BB.Math.Quaternion.t()`
  - `angular_velocity` - Angular velocity as `BB.Math.Vec3.t()` in rad/s
  - `linear_acceleration` - Linear acceleration as `BB.Math.Vec3.t()` in m/s²

  ## Optional uncertainty fields

  Drivers with known noise characteristics may populate per-channel
  covariance matrices. Algorithms that consume them check for `nil` and
  either fall back to a configured default or refuse the reading.
  Mirrors the ROS `sensor_msgs/Imu` shape.

  - `orientation_covariance` - `BB.Math.Covariance3.t() | nil`
  - `angular_velocity_covariance` - `BB.Math.Covariance3.t() | nil`
  - `linear_acceleration_covariance` - `BB.Math.Covariance3.t() | nil`

  ## Examples

      alias BB.Message.Sensor.Imu
      alias BB.Math.{Vec3, Quaternion}

      {:ok, msg} = Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
      )
  """

  import BB.Message.Option

  alias BB.Math.Covariance3
  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  defstruct [
    :orientation,
    :angular_velocity,
    :linear_acceleration,
    :orientation_covariance,
    :angular_velocity_covariance,
    :linear_acceleration_covariance
  ]

  use BB.Message,
    schema: [
      orientation: [type: quaternion_type(), required: true, doc: "Orientation as quaternion"],
      angular_velocity: [type: vec3_type(), required: true, doc: "Angular velocity in rad/s"],
      linear_acceleration: [
        type: vec3_type(),
        required: true,
        doc: "Linear acceleration in m/s²"
      ],
      orientation_covariance: [
        type: {:or, [covariance3_type(), nil]},
        required: false,
        default: nil,
        doc: "3x3 covariance over orientation, or nil if unknown"
      ],
      angular_velocity_covariance: [
        type: {:or, [covariance3_type(), nil]},
        required: false,
        default: nil,
        doc: "3x3 covariance over angular velocity, or nil if unknown"
      ],
      linear_acceleration_covariance: [
        type: {:or, [covariance3_type(), nil]},
        required: false,
        default: nil,
        doc: "3x3 covariance over linear acceleration, or nil if unknown"
      ]
    ]

  @type t :: %__MODULE__{
          orientation: Quaternion.t(),
          angular_velocity: Vec3.t(),
          linear_acceleration: Vec3.t(),
          orientation_covariance: nil | Covariance3.t(),
          angular_velocity_covariance: nil | Covariance3.t(),
          linear_acceleration_covariance: nil | Covariance3.t()
        }
end
