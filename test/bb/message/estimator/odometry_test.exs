# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Estimator.OdometryTest do
  use ExUnit.Case, async: true

  alias BB.Math.Covariance6
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Message.Estimator.Odometry
  alias BB.Message.Geometry.Twist

  describe "new/2" do
    test "constructs with pose, twist, and both covariances" do
      twist = %Twist{linear: Vec3.zero(), angular: Vec3.zero()}

      {:ok, msg} =
        Odometry.new(:base_link,
          pose: Transform.identity(),
          twist: twist,
          pose_covariance: Covariance6.identity(),
          twist_covariance: Covariance6.identity()
        )

      assert %Odometry{} = msg.payload
      assert %Transform{} = msg.payload.pose
      assert %Twist{} = msg.payload.twist
    end

    test "covariances default to nil" do
      twist = %Twist{linear: Vec3.zero(), angular: Vec3.zero()}

      {:ok, msg} =
        Odometry.new(:base_link, pose: Transform.identity(), twist: twist)

      assert msg.payload.pose_covariance == nil
      assert msg.payload.twist_covariance == nil
    end
  end
end
