# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Transform do
  @moduledoc """
  A transformation in 3D space (translation and rotation).

  Represents the relationship between two coordinate frames.

  ## Fields

  - `translation` - Translation as `BB.Vec3.t()` in metres
  - `rotation` - Rotation as `BB.Quaternion.t()`

  ## Examples

      alias BB.Message.Geometry.Transform
      alias BB.{Vec3, Quaternion}

      {:ok, msg} = Transform.new(:base_link, Vec3.new(0.0, 0.0, 1.0), Quaternion.identity())
  """

  import BB.Message.Option

  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  defstruct [:translation, :rotation]

  use BB.Message,
    schema: [
      translation: [type: vec3_type(), required: true, doc: "Translation in metres"],
      rotation: [type: quaternion_type(), required: true, doc: "Rotation as quaternion"]
    ]

  @type t :: %__MODULE__{
          translation: Vec3.t(),
          rotation: Quaternion.t()
        }

  @doc """
  Create a new Transform message.

  Returns `{:ok, %BB.Message{}}` with the transform as payload.

  ## Examples

      alias BB.{Vec3, Quaternion}

      {:ok, msg} = Transform.new(:base_link, Vec3.new(0.0, 0.0, 1.0), Quaternion.identity())
  """
  @spec new(atom(), Vec3.t(), Quaternion.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = translation, %Quaternion{} = rotation) do
    new(frame_id, translation: translation, rotation: rotation)
  end
end
