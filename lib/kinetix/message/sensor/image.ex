defmodule Kinetix.Message.Sensor.Image do
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

      alias Kinetix.Message.Sensor.Image

      {:ok, msg} = Image.new(:camera,
        height: 480,
        width: 640,
        encoding: :rgb8,
        is_bigendian: false,
        step: 1920,
        data: <<0, 0, 0, ...>>
      )
  """

  @behaviour Kinetix.Message

  defstruct [:height, :width, :encoding, :is_bigendian, :step, :data]

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

  @schema Spark.Options.new!(
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
          )

  @doc false
  def validate_binary(value, _opts) when is_binary(value), do: {:ok, value}
  def validate_binary(value, _opts), do: {:error, "expected binary, got: #{inspect(value)}"}

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Image message.

  Returns `{:ok, %Kinetix.Message{}}` with the image as payload.

  ## Examples

      {:ok, msg} = Image.new(:camera,
        height: 480,
        width: 640,
        encoding: :rgb8,
        step: 1920,
        data: <<0::size(921600 * 8)>>
      )
  """
  @spec new(atom(), keyword()) :: {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    Kinetix.Message.new(__MODULE__, frame_id, attrs)
  end
end
