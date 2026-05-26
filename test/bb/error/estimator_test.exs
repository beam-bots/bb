# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.EstimatorTest do
  use ExUnit.Case, async: true

  alias BB.Error.Estimator.MissingCovariance
  alias BB.Error.Estimator.StaleInput
  alias BB.Error.Estimator.SyncMiss

  describe "StaleInput" do
    test "carries input_path, age_ms, budget_ms" do
      err = StaleInput.exception(input_path: [:sensor, :imu], age_ms: 42, budget_ms: 20)
      assert err.input_path == [:sensor, :imu]
      assert err.age_ms == 42
      assert err.budget_ms == 20
    end

    test "is a warning" do
      err = StaleInput.exception(input_path: [], age_ms: 1, budget_ms: 0)
      assert BB.Error.severity(err) == :warning
    end

    test "message includes path and ages" do
      err = StaleInput.exception(input_path: [:sensor, :imu], age_ms: 42, budget_ms: 20)
      message = StaleInput.message(err)
      assert message =~ "[:sensor, :imu]"
      assert message =~ "42"
      assert message =~ "20"
    end
  end

  describe "SyncMiss" do
    test "carries gap and tolerance" do
      err =
        SyncMiss.exception(
          driver_path: [:sensor, :imu],
          input_path: [:sensor, :odom],
          gap_ms: 150,
          tolerance_ms: 50
        )

      assert err.gap_ms == 150
      assert err.tolerance_ms == 50
    end

    test "is a warning" do
      err =
        SyncMiss.exception(driver_path: [], input_path: [], gap_ms: 0, tolerance_ms: 0)

      assert BB.Error.severity(err) == :warning
    end
  end

  describe "MissingCovariance" do
    test "carries estimator and field" do
      err = MissingCovariance.exception(estimator: :pose, field: :orientation_covariance)
      assert err.estimator == :pose
      assert err.field == :orientation_covariance
    end

    test "is an error" do
      err = MissingCovariance.exception(estimator: :pose, field: :orientation_covariance)
      assert BB.Error.severity(err) == :error
    end
  end
end
