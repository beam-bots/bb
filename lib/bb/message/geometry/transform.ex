# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Transform do
  @moduledoc """
  A transformation in 3D space (translation and rotation).

  Represents the relationship between two coordinate frames.

  ## Fields

  - `translation` - Translation as `{:vec3, x, y, z}` in metres
  - `rotation` - Rotation as `{:quaternion, x, y, z, w}`

  ## Examples

      alias BB.Message.Geometry.Transform
      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = Transform.new(:base_link, Vec3.new(0.0, 0.0, 1.0), Quaternion.identity())
  """

  @behaviour BB.Message

  import BB.Message.Option

  defstruct [:translation, :rotation]

  @type t :: %__MODULE__{
          translation: BB.Message.Vec3.t(),
          rotation: BB.Message.Quaternion.t()
        }

  @schema Spark.Options.new!(
            translation: [type: vec3_type(), required: true, doc: "Translation in metres"],
            rotation: [type: quaternion_type(), required: true, doc: "Rotation as quaternion"]
          )

  @impl BB.Message
  def schema, do: @schema

  defimpl BB.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Transform message.

  Returns `{:ok, %BB.Message{}}` with the transform as payload.

  ## Examples

      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = Transform.new(:base_link, Vec3.new(0.0, 0.0, 1.0), Quaternion.identity())
  """
  @spec new(atom(), BB.Message.Vec3.t(), BB.Message.Quaternion.t()) ::
          {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = translation, {:quaternion, _, _, _, _} = rotation) do
    BB.Message.new(__MODULE__, frame_id,
      translation: translation,
      rotation: rotation
    )
  end
end
