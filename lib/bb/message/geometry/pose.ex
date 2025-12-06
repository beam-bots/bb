# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Pose do
  @moduledoc """
  A position and orientation in 3D space.

  ## Fields

  - `position` - Position as `{:vec3, x, y, z}` in metres
  - `orientation` - Orientation as `{:quaternion, x, y, z, w}`

  ## Examples

      alias BB.Message.Geometry.Pose
      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:end_effector, Vec3.new(1.0, 0.0, 0.5), Quaternion.identity())
  """

  @behaviour BB.Message

  import BB.Message.Option

  defstruct [:position, :orientation]

  @type t :: %__MODULE__{
          position: BB.Message.Vec3.t(),
          orientation: BB.Message.Quaternion.t()
        }

  @schema Spark.Options.new!(
            position: [type: vec3_type(), required: true, doc: "Position in metres"],
            orientation: [
              type: quaternion_type(),
              required: true,
              doc: "Orientation as quaternion"
            ]
          )

  @impl BB.Message
  def schema, do: @schema

  defimpl BB.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Pose message.

  Returns `{:ok, %BB.Message{}}` with the pose as payload.

  ## Examples

      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:base_link, Vec3.new(1.0, 2.0, 3.0), Quaternion.identity())
  """
  @spec new(atom(), BB.Message.Vec3.t(), BB.Message.Quaternion.t()) ::
          {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = position, {:quaternion, _, _, _, _} = orientation) do
    BB.Message.new(__MODULE__, frame_id,
      position: position,
      orientation: orientation
    )
  end
end
