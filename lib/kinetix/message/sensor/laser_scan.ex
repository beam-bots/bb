defmodule Kinetix.Message.Sensor.LaserScan do
  @moduledoc """
  Single scan from a planar laser range-finder.

  ## Fields

  - `angle_min` - Start angle of scan in radians
  - `angle_max` - End angle of scan in radians
  - `angle_increment` - Angular distance between measurements in radians
  - `time_increment` - Time between measurements in seconds
  - `scan_time` - Time between scans in seconds
  - `range_min` - Minimum range value in metres
  - `range_max` - Maximum range value in metres
  - `ranges` - Range data in metres (values < range_min or > range_max are invalid)
  - `intensities` - Intensity data (device-specific units, optional)

  ## Examples

      alias Kinetix.Message.Sensor.LaserScan

      {:ok, msg} = LaserScan.new(:laser_frame,
        angle_min: -1.57,
        angle_max: 1.57,
        angle_increment: 0.01,
        time_increment: 0.0001,
        scan_time: 0.1,
        range_min: 0.1,
        range_max: 10.0,
        ranges: [1.0, 1.1, 1.2, 1.3]
      )
  """

  @behaviour Kinetix.Message

  defstruct [
    :angle_min,
    :angle_max,
    :angle_increment,
    :time_increment,
    :scan_time,
    :range_min,
    :range_max,
    :ranges,
    :intensities
  ]

  @type t :: %__MODULE__{
          angle_min: float(),
          angle_max: float(),
          angle_increment: float(),
          time_increment: float(),
          scan_time: float(),
          range_min: float(),
          range_max: float(),
          ranges: [float()],
          intensities: [float()]
        }

  @schema Spark.Options.new!(
            angle_min: [type: :float, required: true, doc: "Start angle in radians"],
            angle_max: [type: :float, required: true, doc: "End angle in radians"],
            angle_increment: [
              type: :float,
              required: true,
              doc: "Angular distance between measurements in radians"
            ],
            time_increment: [
              type: :float,
              required: true,
              doc: "Time between measurements in seconds"
            ],
            scan_time: [type: :float, required: true, doc: "Time between scans in seconds"],
            range_min: [type: :float, required: true, doc: "Minimum range in metres"],
            range_max: [type: :float, required: true, doc: "Maximum range in metres"],
            ranges: [type: {:list, :float}, required: true, doc: "Range data in metres"],
            intensities: [type: {:list, :float}, default: [], doc: "Intensity data (optional)"]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new LaserScan message.

  Returns `{:ok, %Kinetix.Message{}}` with the laser scan as payload.
  """
  @spec new(atom(), keyword()) :: {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    Kinetix.Message.new(__MODULE__, frame_id, attrs)
  end
end
