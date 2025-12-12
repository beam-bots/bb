# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.EndMotionTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Actuator.EndMotion

  describe "EndMotion" do
    test "creates an end motion message with required fields only" do
      {:ok, msg} =
        Message.new(EndMotion, :shoulder,
          position: 1.57,
          reason: :completed
        )

      assert msg.frame_id == :shoulder
      assert msg.payload.position == 1.57
      assert msg.payload.reason == :completed
      assert msg.payload.detail == nil
      assert msg.payload.message == nil
    end

    test "creates an end motion message with detail" do
      {:ok, msg} =
        Message.new(EndMotion, :shoulder,
          position: 0.0,
          reason: :limit_reached,
          detail: :end_stop
        )

      assert msg.payload.reason == :limit_reached
      assert msg.payload.detail == :end_stop
    end

    test "creates an end motion message with message" do
      {:ok, msg} =
        Message.new(EndMotion, :shoulder,
          position: 0.52,
          reason: :fault,
          detail: :stall,
          message: "Motor stall detected at 30% travel"
        )

      assert msg.payload.reason == :fault
      assert msg.payload.detail == :stall
      assert msg.payload.message == "Motor stall detected at 30% travel"
    end

    test "requires position" do
      assert {:error, _} =
               Message.new(EndMotion, :shoulder, reason: :completed)
    end

    test "requires reason" do
      assert {:error, _} =
               Message.new(EndMotion, :shoulder, position: 1.57)
    end

    test "validates reason is one of the allowed values" do
      assert {:error, _} =
               Message.new(EndMotion, :shoulder,
                 position: 1.57,
                 reason: :invalid_reason
               )
    end

    test "accepts all valid reasons" do
      for reason <- [:completed, :cancelled, :limit_reached, :fault] do
        {:ok, msg} =
          Message.new(EndMotion, :shoulder,
            position: 1.57,
            reason: reason
          )

        assert msg.payload.reason == reason
      end
    end

    test "new!/3 raises on validation error" do
      assert_raise Spark.Options.ValidationError, fn ->
        Message.new!(EndMotion, :shoulder, position: 1.57, reason: :invalid)
      end
    end
  end
end
