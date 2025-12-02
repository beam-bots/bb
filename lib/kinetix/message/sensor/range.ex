defmodule Kinetix.Message.Sensor.Range do
  @moduledoc """
  Single range reading from a distance sensor.

  ## Fields

  - `radiation_type` - Type of radiation (`:ultrasound` or `:infrared`)
  - `field_of_view` - Size of the arc that the sensor covers in radians
  - `min_range` - Minimum range in metres
  - `max_range` - Maximum range in metres
  - `range` - Measured range in metres

  Values less than min_range or greater than max_range should be discarded.
  A range of `:infinity` indicates no object was detected.

  ## Examples

      alias Kinetix.Message.Sensor.Range

      {:ok, msg} = Range.new(:ultrasonic_sensor,
        radiation_type: :ultrasound,
        field_of_view: 0.26,
        min_range: 0.02,
        max_range: 4.0,
        range: 1.5
      )
  """

  @behaviour Kinetix.Message

  defstruct [:radiation_type, :field_of_view, :min_range, :max_range, :range]

  @type radiation_type :: :ultrasound | :infrared
  @type t :: %__MODULE__{
          radiation_type: radiation_type(),
          field_of_view: float(),
          min_range: float(),
          max_range: float(),
          range: float() | :infinity
        }

  @schema Spark.Options.new!(
            radiation_type: [
              type: {:in, [:ultrasound, :infrared]},
              required: true,
              doc: "Type of radiation"
            ],
            field_of_view: [
              type: :float,
              required: true,
              doc: "Size of the arc in radians"
            ],
            min_range: [type: :float, required: true, doc: "Minimum range in metres"],
            max_range: [type: :float, required: true, doc: "Maximum range in metres"],
            range: [
              type: {:or, [:float, {:literal, :infinity}]},
              required: true,
              doc: "Measured range in metres or :infinity"
            ]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Range message.

  Returns `{:ok, %Kinetix.Message{}}` with the range data as payload.

  ## Examples

      {:ok, msg} = Range.new(:ultrasonic_sensor,
        radiation_type: :ultrasound,
        field_of_view: 0.26,
        min_range: 0.02,
        max_range: 4.0,
        range: 1.5
      )
  """
  @spec new(atom(), keyword()) :: {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    Kinetix.Message.new(__MODULE__, frame_id, attrs)
  end
end
