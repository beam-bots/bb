# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.ImageTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Sensor.Image

  test "creates an image message" do
    data = <<0, 0, 0, 255, 255, 255>>

    {:ok, msg} =
      Image.new(:camera,
        height: 1,
        width: 2,
        encoding: :rgb8,
        step: 6,
        data: data
      )

    assert %Message{payload: %Image{}} = msg
    assert msg.payload.height == 1
    assert msg.payload.width == 2
    assert msg.payload.is_bigendian == false
  end
end
