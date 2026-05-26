# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Estimator.Odometry do
  @moduledoc """
  An odometry estimate: pose + twist with separate covariances.

  Mirrors ROS `nav_msgs/Odometry`. The canonical output for IMU + wheel
  odometry EKFs and any other fusion stage that publishes both a pose
  and a velocity simultaneously.

  ## Fields

  - `pose` - Pose component as `BB.Math.Transform.t()`.
  - `twist` - Linear and angular velocity as `BB.Message.Geometry.Twist.t()`.
  - `pose_covariance` - Optional `BB.Math.Covariance6.t()` over the pose.
  - `twist_covariance` - Optional `BB.Math.Covariance6.t()` over the twist.

  ## Examples

      alias BB.Message.Estimator.Odometry
      alias BB.Message.Geometry.Twist
      alias BB.Math.{Transform, Vec3, Covariance6}

      twist_payload = %Twist{linear: Vec3.zero(), angular: Vec3.zero()}

      {:ok, msg} =
        Odometry.new(:base_link,
          pose: Transform.identity(),
          twist: twist_payload,
          pose_covariance: Covariance6.identity(),
          twist_covariance: nil
        )
  """

  import BB.Message.Option

  alias BB.Math.Covariance6
  alias BB.Math.Transform
  alias BB.Message.Geometry.Twist

  defstruct [:pose, :twist, :pose_covariance, :twist_covariance]

  use BB.Message,
    schema: [
      pose: [type: transform_type(), required: true, doc: "Pose component as Transform"],
      twist: [
        type: {:struct, BB.Message.Geometry.Twist},
        required: true,
        doc: "Linear and angular velocity as a Twist payload"
      ],
      pose_covariance: [
        type: {:or, [covariance6_type(), nil]},
        required: false,
        default: nil,
        doc: "6x6 covariance over the pose, or nil if unknown"
      ],
      twist_covariance: [
        type: {:or, [covariance6_type(), nil]},
        required: false,
        default: nil,
        doc: "6x6 covariance over the twist, or nil if unknown"
      ]
    ]

  @type t :: %__MODULE__{
          pose: Transform.t(),
          twist: Twist.t(),
          pose_covariance: nil | Covariance6.t(),
          twist_covariance: nil | Covariance6.t()
        }
end
