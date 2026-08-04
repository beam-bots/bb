# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Wrench2DTest do
  use ExUnit.Case, async: true
  doctest BB.Message.Geometry.Wrench2D

  alias BB.Message
  alias BB.Message.Geometry.Wrench2D

  test "creates a wrench message" do
    {:ok, msg} = Wrench2D.new(:chassis, 12.0, 0.0, 0.4)

    assert %Message{payload: %Wrench2D{fx: 12.0, fy: +0.0, tau: 0.4}} = msg
    assert msg.frame_id == :chassis
  end

  test "coerces integers to floats" do
    {:ok, msg} = Wrench2D.new(:chassis, 1, 2, 3)

    assert %Message{payload: %Wrench2D{fx: 1.0, fy: 2.0, tau: 3.0}} = msg
  end

  test "requires every component" do
    assert {:error, _} = Wrench2D.new(:chassis, fx: 12.0, fy: 0.0)
  end
end
