defmodule Kinetix.Message.Geometry.Transform do
  @moduledoc """
  A transformation in 3D space (translation and rotation).

  Represents the relationship between two coordinate frames.

  ## Fields

  - `translation` - Translation as `{:vec3, x, y, z}` in metres
  - `rotation` - Rotation as `{:quaternion, x, y, z, w}`

  ## Examples

      alias Kinetix.Message.Geometry.Transform
      alias Kinetix.Message.{Vec3, Quaternion}

      {:ok, msg} = Transform.new(:base_link, Vec3.new(0.0, 0.0, 1.0), Quaternion.identity())
  """

  @behaviour Kinetix.Message

  import Kinetix.Message.Option

  defstruct [:translation, :rotation]

  @type t :: %__MODULE__{
          translation: Kinetix.Message.Vec3.t(),
          rotation: Kinetix.Message.Quaternion.t()
        }

  @schema Spark.Options.new!(
            translation: [type: vec3_type(), required: true, doc: "Translation in metres"],
            rotation: [type: quaternion_type(), required: true, doc: "Rotation as quaternion"]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Transform message.

  Returns `{:ok, %Kinetix.Message{}}` with the transform as payload.

  ## Examples

      alias Kinetix.Message.{Vec3, Quaternion}

      {:ok, msg} = Transform.new(:base_link, Vec3.new(0.0, 0.0, 1.0), Quaternion.identity())
  """
  @spec new(atom(), Kinetix.Message.Vec3.t(), Kinetix.Message.Quaternion.t()) ::
          {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = translation, {:quaternion, _, _, _, _} = rotation) do
    Kinetix.Message.new(__MODULE__, frame_id,
      translation: translation,
      rotation: rotation
    )
  end
end
