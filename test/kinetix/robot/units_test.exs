# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Robot.UnitsTest do
  use ExUnit.Case, async: true
  import Kinetix.Unit

  alias Kinetix.Robot.Units

  describe "to_meters/1" do
    test "converts meters to float" do
      assert Units.to_meters(~u(5 meter)) == 5.0
    end

    test "converts centimeters to meters" do
      assert Units.to_meters(~u(100 centimeter)) == 1.0
    end

    test "handles float values" do
      assert Units.to_meters(~u(1.5 meter)) == 1.5
    end

    test "handles negative values" do
      assert Units.to_meters(~u(-2 meter)) == -2.0
    end
  end

  describe "to_radians/1" do
    test "converts degrees to radians" do
      result = Units.to_radians(~u(180 degree))
      assert_in_delta result, :math.pi(), 0.0001
    end

    test "converts 90 degrees correctly" do
      result = Units.to_radians(~u(90 degree))
      assert_in_delta result, :math.pi() / 2, 0.0001
    end

    test "handles negative angles" do
      result = Units.to_radians(~u(-90 degree))
      assert_in_delta result, -:math.pi() / 2, 0.0001
    end

    test "converts radians to radians" do
      result = Units.to_radians(~u(1 radian))
      assert_in_delta result, 1.0, 0.0001
    end
  end

  describe "to_kilograms/1" do
    test "converts kilograms to float" do
      assert Units.to_kilograms(~u(5 kilogram)) == 5.0
    end

    test "converts grams to kilograms" do
      assert Units.to_kilograms(~u(1000 gram)) == 1.0
    end
  end

  describe "to_kilogram_square_meters/1" do
    test "converts moment of inertia" do
      assert Units.to_kilogram_square_meters(~u(0.5 kilogram_square_meter)) == 0.5
    end
  end

  describe "to_newtons/1" do
    test "converts force to float" do
      assert Units.to_newtons(~u(10 newton)) == 10.0
    end
  end

  describe "to_newton_meters/1" do
    test "converts torque to float" do
      assert Units.to_newton_meters(~u(5 newton_meter)) == 5.0
    end
  end

  describe "to_meters_per_second/1" do
    test "converts linear velocity" do
      assert Units.to_meters_per_second(~u(10 meter_per_second)) == 10.0
    end
  end

  describe "to_radians_per_second/1" do
    test "converts angular velocity" do
      result = Units.to_radians_per_second(~u(180 degree_per_second))
      assert_in_delta result, :math.pi(), 0.0001
    end
  end

  describe "to_linear_damping/1" do
    test "converts linear damping coefficient" do
      assert Units.to_linear_damping(~u(1.5 newton_second_per_meter)) == 1.5
    end
  end

  describe "optional conversions" do
    test "to_meters_or_nil/1 returns nil for nil" do
      assert Units.to_meters_or_nil(nil) == nil
    end

    test "to_meters_or_nil/1 converts non-nil" do
      assert Units.to_meters_or_nil(~u(1 meter)) == 1.0
    end

    test "to_radians_or_nil/1 returns nil for nil" do
      assert Units.to_radians_or_nil(nil) == nil
    end

    test "to_radians_or_nil/1 converts non-nil" do
      result = Units.to_radians_or_nil(~u(90 degree))
      assert_in_delta result, :math.pi() / 2, 0.0001
    end
  end

  describe "extract_float/1" do
    test "extracts integer value" do
      unit = Cldr.Unit.new!(:meter, 5)
      assert Units.extract_float(unit) == 5.0
    end

    test "extracts float value" do
      unit = Cldr.Unit.new!(:meter, 2.5)
      assert Units.extract_float(unit) == 2.5
    end

    test "extracts decimal value" do
      unit = Cldr.Unit.new!(:meter, Decimal.new("1.5"))
      assert Units.extract_float(unit) == 1.5
    end
  end
end
