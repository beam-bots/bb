# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.BeginMotionTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion

  describe "BeginMotion" do
    test "creates a begin motion message" do
      expected_arrival = System.monotonic_time(:millisecond) + 500

      {:ok, msg} =
        Message.new(BeginMotion, :shoulder,
          initial_position: 0.0,
          target_position: 1.57,
          expected_arrival: expected_arrival
        )

      assert msg.frame_id == :shoulder
      assert msg.payload.initial_position == 0.0
      assert msg.payload.target_position == 1.57
      assert msg.payload.expected_arrival == expected_arrival
    end

    test "requires initial_position" do
      assert {:error, _} =
               Message.new(BeginMotion, :shoulder,
                 target_position: 1.57,
                 expected_arrival: 1000
               )
    end

    test "requires target_position" do
      assert {:error, _} =
               Message.new(BeginMotion, :shoulder,
                 initial_position: 0.0,
                 expected_arrival: 1000
               )
    end

    test "requires expected_arrival" do
      assert {:error, _} =
               Message.new(BeginMotion, :shoulder,
                 initial_position: 0.0,
                 target_position: 1.57
               )
    end

    test "new!/3 raises on validation error" do
      assert_raise Spark.Options.ValidationError, fn ->
        Message.new!(BeginMotion, :shoulder, initial_position: "invalid")
      end
    end
  end
end
