# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Actuator.MotorProfileTest do
  use ExUnit.Case, async: true

  alias BB.Actuator.MotorProfile

  describe "from_joint/2" do
    test "nil joint yields an empty profile centred at zero" do
      profile = MotorProfile.from_joint(nil, nil)

      assert profile.motor_lower == nil
      assert profile.motor_upper == nil
      assert profile.motor_velocity_limit == nil
      assert profile.motor_acceleration_limit == nil
      assert profile.motor_effort_limit == nil
      assert profile.motor_initial_position == 0.0
    end

    test "joint without limits yields an empty profile centred at zero" do
      profile = MotorProfile.from_joint(%{limits: nil}, nil)

      assert profile.motor_lower == nil
      assert profile.motor_initial_position == 0.0
    end

    test "no transmission passes limits straight through" do
      joint = %{
        limits: %{lower: -1.0, upper: 2.0, velocity: 5.0, acceleration: 10.0, effort: 0.5}
      }

      profile = MotorProfile.from_joint(joint, nil)

      assert profile.motor_lower == -1.0
      assert profile.motor_upper == 2.0
      assert profile.motor_velocity_limit == 5.0
      assert profile.motor_acceleration_limit == 10.0
      assert profile.motor_effort_limit == 0.5
      assert profile.motor_initial_position == 0.5
    end

    test "applies a forward transmission to position limits" do
      joint = %{limits: %{lower: -1.0, upper: 2.0, velocity: nil, acceleration: nil, effort: nil}}
      transmission = %{reduction: 10.0, offset: 0.5, reversed?: false}

      profile = MotorProfile.from_joint(joint, transmission)

      assert_in_delta profile.motor_lower, -15.0, 1.0e-9
      assert_in_delta profile.motor_upper, 15.0, 1.0e-9
      assert_in_delta profile.motor_initial_position, 0.0, 1.0e-9
    end

    test "reversed transmission swaps lower/upper after the sign flip" do
      joint = %{limits: %{lower: 0.0, upper: 1.0, velocity: nil, acceleration: nil, effort: nil}}
      transmission = %{reduction: 1.0, offset: 0.0, reversed?: true}

      profile = MotorProfile.from_joint(joint, transmission)

      assert profile.motor_lower == -1.0
      assert profile.motor_upper == 0.0
      assert profile.motor_initial_position == -0.5
    end

    test "velocity, acceleration and effort are positive magnitudes" do
      joint = %{
        limits: %{lower: nil, upper: nil, velocity: 5.0, acceleration: 10.0, effort: 0.5}
      }

      transmission = %{reduction: 50.0, offset: 0.0, reversed?: true}
      profile = MotorProfile.from_joint(joint, transmission)

      assert profile.motor_velocity_limit > 0.0
      assert profile.motor_acceleration_limit > 0.0
      assert profile.motor_effort_limit > 0.0
    end

    test "nil position limits leave initial position at zero" do
      joint = %{
        limits: %{lower: nil, upper: nil, velocity: 5.0, acceleration: nil, effort: nil}
      }

      assert MotorProfile.from_joint(joint, nil).motor_initial_position == 0.0
    end
  end
end
