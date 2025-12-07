# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.Image do
  @moduledoc """
  Raw image data from a camera sensor.

  ## Fields

  - `height` - Image height in pixels
  - `width` - Image width in pixels
  - `encoding` - Pixel encoding format
  - `is_bigendian` - Whether data is big-endian
  - `step` - Full row length in bytes
  - `data` - Actual image data as binary

  ## Encodings

  Common encodings include:
  - `:rgb8` - RGB 8-bit per channel
  - `:rgba8` - RGBA 8-bit per channel
  - `:bgr8` - BGR 8-bit per channel
  - `:bgra8` - BGRA 8-bit per channel
  - `:mono8` - Grayscale 8-bit
  - `:mono16` - Grayscale 16-bit

  ## Examples

      alias BB.Message.Sensor.Image

      {:ok, msg} = Image.new(:camera,
        height: 480,
        width: 640,
        encoding: :rgb8,
        is_bigendian: false,
        step: 1920,
        data: <<0, 0, 0, ...>>
      )
  """

  @encodings [
    :rgb8,
    :rgba8,
    :rgb16,
    :rgba16,
    :bgr8,
    :bgra8,
    :bgr16,
    :bgra16,
    :mono8,
    :mono16,
    :bayer_rggb8,
    :bayer_bggr8,
    :bayer_gbrg8,
    :bayer_grbg8
  ]

  defstruct [:height, :width, :encoding, :is_bigendian, :step, :data]

  use BB.Message,
    schema: [
      height: [type: :non_neg_integer, required: true, doc: "Image height in pixels"],
      width: [type: :non_neg_integer, required: true, doc: "Image width in pixels"],
      encoding: [type: {:in, @encodings}, required: true, doc: "Pixel encoding format"],
      is_bigendian: [type: :boolean, default: false, doc: "Whether data is big-endian"],
      step: [type: :non_neg_integer, required: true, doc: "Full row length in bytes"],
      data: [
        type: {:custom, __MODULE__, :validate_binary, [[]]},
        required: true,
        doc: "Image data as binary"
      ]
    ]

  @type encoding ::
          :rgb8
          | :rgba8
          | :rgb16
          | :rgba16
          | :bgr8
          | :bgra8
          | :bgr16
          | :bgra16
          | :mono8
          | :mono16
          | :bayer_rggb8
          | :bayer_bggr8
          | :bayer_gbrg8
          | :bayer_grbg8

  @type t :: %__MODULE__{
          height: non_neg_integer(),
          width: non_neg_integer(),
          encoding: encoding(),
          is_bigendian: boolean(),
          step: non_neg_integer(),
          data: binary()
        }

  @doc false
  def validate_binary(value, _opts) when is_binary(value), do: {:ok, value}
  def validate_binary(value, _opts), do: {:error, "expected binary, got: #{inspect(value)}"}
end
