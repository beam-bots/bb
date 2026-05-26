# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Estimator.Pose do
  @moduledoc """
  A 6-DOF pose estimate with optional covariance.

  The canonical output payload for any estimator that publishes a fused
  pose (e.g. an AHRS, an EKF combining IMU and odometry, a visual SLAM
  front-end). Subscribers wanting fused poses specifically can filter on
  this payload type to distinguish them from waypoints
  (`BB.Message.Geometry.Pose`) or command goals.

  ## Fields

  - `transform` - The pose as `BB.Math.Transform.t()` in the frame named
    by the wrapping message's `:frame_id`.
  - `covariance` - Optional `BB.Math.Covariance6.t()` over the pose's
    6 DOF (translation x/y/z, rotation r/p/y). `nil` if the estimator
    does not produce a covariance.

  ## Examples

      alias BB.Message.Estimator.Pose
      alias BB.Math.{Transform, Covariance6}

      {:ok, msg} =
        Pose.new(:base_link,
          transform: Transform.identity(),
          covariance: Covariance6.diagonal([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        )
  """

  import BB.Message.Option

  alias BB.Math.Covariance6
  alias BB.Math.Transform

  defstruct [:transform, :covariance]

  use BB.Message,
    schema: [
      transform: [type: transform_type(), required: true, doc: "Pose as Transform"],
      covariance: [
        type: {:or, [covariance6_type(), nil]},
        required: false,
        default: nil,
        doc: "6x6 covariance over translation and rotation, or nil if unknown"
      ]
    ]

  @type t :: %__MODULE__{
          transform: Transform.t(),
          covariance: nil | Covariance6.t()
        }
end
