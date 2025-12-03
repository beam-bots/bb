# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Message.Geometry.Pose do
  @moduledoc """
  A position and orientation in 3D space.

  ## Fields

  - `position` - Position as `{:vec3, x, y, z}` in metres
  - `orientation` - Orientation as `{:quaternion, x, y, z, w}`

  ## Examples

      alias Kinetix.Message.Geometry.Pose
      alias Kinetix.Message.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:end_effector, Vec3.new(1.0, 0.0, 0.5), Quaternion.identity())
  """

  @behaviour Kinetix.Message

  import Kinetix.Message.Option

  defstruct [:position, :orientation]

  @type t :: %__MODULE__{
          position: Kinetix.Message.Vec3.t(),
          orientation: Kinetix.Message.Quaternion.t()
        }

  @schema Spark.Options.new!(
            position: [type: vec3_type(), required: true, doc: "Position in metres"],
            orientation: [
              type: quaternion_type(),
              required: true,
              doc: "Orientation as quaternion"
            ]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Pose message.

  Returns `{:ok, %Kinetix.Message{}}` with the pose as payload.

  ## Examples

      alias Kinetix.Message.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:base_link, Vec3.new(1.0, 2.0, 3.0), Quaternion.identity())
  """
  @spec new(atom(), Kinetix.Message.Vec3.t(), Kinetix.Message.Quaternion.t()) ::
          {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = position, {:quaternion, _, _, _, _} = orientation) do
    Kinetix.Message.new(__MODULE__, frame_id,
      position: position,
      orientation: orientation
    )
  end
end
