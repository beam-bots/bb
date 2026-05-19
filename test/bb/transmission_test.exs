# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.TransmissionTest do
  use ExUnit.Case, async: true

  alias BB.Transmission

  @identity %{reduction: 1.0, offset: 0.0, reversed?: false}

  describe "apply_position/2" do
    test "identity transmission is a no-op" do
      assert Transmission.apply_position(0.5, @identity) == 0.5
      assert Transmission.apply_position(-1.2, @identity) == -1.2
    end

    test "reduction scales the motor angle by the gear ratio" do
      t = %{@identity | reduction: 50.0}
      assert Transmission.apply_position(0.01, t) == 0.5
    end

    test "reversed? flips the sign" do
      t = %{@identity | reversed?: true}
      assert Transmission.apply_position(0.5, t) == -0.5
    end

    test "offset shifts the joint zero" do
      t = %{@identity | offset: 0.1}
      assert_in_delta Transmission.apply_position(0.1, t), 0.0, 1.0e-9
      assert_in_delta Transmission.apply_position(0.3, t), 0.2, 1.0e-9
    end

    test "reduction, offset, and reversed? combine" do
      t = %{reduction: 50.0, offset: 0.1, reversed?: true}
      assert_in_delta Transmission.apply_position(0.3, t), -10.0, 1.0e-9
    end
  end

  describe "unapply_position/2" do
    test "is the left inverse of apply_position/2" do
      t = %{reduction: 50.0, offset: 0.1, reversed?: true}

      for v <- [-1.0, -0.5, 0.0, 0.1, 0.5, 1.0, 3.14] do
        assert_in_delta Transmission.unapply_position(Transmission.apply_position(v, t), t),
                        v,
                        1.0e-9
      end
    end

    test "is the right inverse of apply_position/2" do
      t = %{reduction: 30.0, offset: -0.05, reversed?: false}

      for v <- [-100.0, -1.0, 0.0, 1.0, 100.0] do
        assert_in_delta Transmission.apply_position(Transmission.unapply_position(v, t), t),
                        v,
                        1.0e-9
      end
    end
  end

  describe "apply_rate/2 and unapply_rate/2" do
    test "rate has no offset component" do
      t = %{reduction: 50.0, offset: 100.0, reversed?: false}
      assert Transmission.apply_rate(0.01, t) == 0.5
      assert Transmission.unapply_rate(0.5, t) == 0.01
    end

    test "reduction scales rate" do
      t = %{@identity | reduction: 50.0}
      assert Transmission.apply_rate(0.1, t) == 5.0
      assert Transmission.unapply_rate(5.0, t) == 0.1
    end

    test "reversed? flips sign" do
      t = %{@identity | reversed?: true, reduction: 2.0}
      assert Transmission.apply_rate(1.0, t) == -2.0
      assert Transmission.unapply_rate(-2.0, t) == 1.0
    end

    test "apply ∘ unapply is identity" do
      t = %{reduction: 17.5, offset: 0.3, reversed?: true}

      for v <- [-3.0, -0.5, 0.0, 0.5, 3.0] do
        assert_in_delta Transmission.unapply_rate(Transmission.apply_rate(v, t), t), v, 1.0e-9
      end
    end
  end

  describe "apply_effort/2 and unapply_effort/2" do
    test "effort divides by reduction (inverse of position)" do
      t = %{@identity | reduction: 50.0}
      assert Transmission.apply_effort(10.0, t) == 0.2
      assert Transmission.unapply_effort(0.2, t) == 10.0
    end

    test "reversed? flips sign for effort too" do
      t = %{@identity | reversed?: true, reduction: 2.0}
      assert Transmission.apply_effort(10.0, t) == -5.0
    end

    test "apply ∘ unapply is identity" do
      t = %{reduction: 17.5, offset: 0.3, reversed?: true}

      for v <- [-3.0, -0.5, 0.0, 0.5, 3.0] do
        assert_in_delta Transmission.unapply_effort(Transmission.apply_effort(v, t), t),
                        v,
                        1.0e-9
      end
    end
  end
end
