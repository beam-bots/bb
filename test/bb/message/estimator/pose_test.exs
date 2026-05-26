# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Estimator.PoseTest do
  use ExUnit.Case, async: true

  alias BB.Math.Covariance6
  alias BB.Math.Transform
  alias BB.Message.Estimator.Pose

  describe "new/2" do
    test "constructs without covariance" do
      {:ok, msg} = Pose.new(:base_link, transform: Transform.identity())

      assert msg.frame_id == :base_link
      assert %Pose{} = msg.payload
      assert msg.payload.covariance == nil
    end

    test "constructs with a covariance" do
      cov = Covariance6.identity()
      {:ok, msg} = Pose.new(:base_link, transform: Transform.identity(), covariance: cov)

      assert %Covariance6{} = msg.payload.covariance
    end

    test "rejects a non-Transform" do
      assert {:error, _} = Pose.new(:base_link, transform: :not_a_transform)
    end

    test "rejects a non-Covariance6 covariance" do
      assert {:error, _} =
               Pose.new(:base_link, transform: Transform.identity(), covariance: :not_a_cov)
    end
  end
end
