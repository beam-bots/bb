# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Twist2DTest do
  use ExUnit.Case, async: true
  doctest BB.Message.Geometry.Twist2D

  alias BB.Message
  alias BB.Message.Geometry.Twist2D

  test "creates a twist message" do
    {:ok, msg} = Twist2D.new(:chassis, 0.5, 0.0, 0.1)

    assert %Message{payload: %Twist2D{vx: 0.5, vy: +0.0, omega: 0.1}} = msg
    assert msg.frame_id == :chassis
  end

  test "coerces integers to floats" do
    {:ok, msg} = Twist2D.new(:chassis, 1, 2, 3)

    assert %Message{payload: %Twist2D{vx: 1.0, vy: 2.0, omega: 3.0}} = msg
  end

  test "requires every component" do
    assert {:error, _} = Twist2D.new(:chassis, vx: 0.5, vy: 0.0)
  end
end
