defmodule Kinetix.Message.Sensor.JointState do
  @moduledoc """
  State of a set of joints.

  ## Fields

  - `names` - List of joint names as atoms
  - `positions` - Joint positions in radians (revolute) or metres (prismatic)
  - `velocities` - Joint velocities in rad/s or m/s
  - `efforts` - Joint efforts in Nm or N

  All lists must have the same length. Missing values can be represented
  as empty lists.

  ## Examples

      alias Kinetix.Message.Sensor.JointState

      {:ok, msg} = JointState.new(:arm,
        names: [:joint1, :joint2],
        positions: [0.0, 1.57],
        velocities: [0.1, 0.0],
        efforts: [0.5, 0.2]
      )
  """

  @behaviour Kinetix.Message

  defstruct [:names, :positions, :velocities, :efforts]

  @type t :: %__MODULE__{
          names: [atom()],
          positions: [float()],
          velocities: [float()],
          efforts: [float()]
        }

  @schema Spark.Options.new!(
            names: [
              type: {:list, :atom},
              required: true,
              doc: "Joint names"
            ],
            positions: [
              type: {:list, :float},
              default: [],
              doc: "Joint positions in radians or metres"
            ],
            velocities: [
              type: {:list, :float},
              default: [],
              doc: "Joint velocities in rad/s or m/s"
            ],
            efforts: [
              type: {:list, :float},
              default: [],
              doc: "Joint efforts in Nm or N"
            ]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new JointState message.

  Returns `{:ok, %Kinetix.Message{}}` with the joint state as payload.

  ## Examples

      {:ok, msg} = JointState.new(:arm,
        names: [:joint1, :joint2],
        positions: [0.0, 1.57]
      )
  """
  @spec new(atom(), keyword()) :: {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    Kinetix.Message.new(__MODULE__, frame_id, attrs)
  end
end
