# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Pose do
  @moduledoc """
  A position and orientation in 3D space.

  ## Fields

  - `position` - Position as `BB.Vec3.t()` in metres
  - `orientation` - Orientation as `BB.Quaternion.t()`

  ## Examples

      alias BB.Message.Geometry.Pose
      alias BB.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:end_effector, Vec3.new(1.0, 0.0, 0.5), Quaternion.identity())
  """

  import BB.Message.Option

  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  defstruct [:position, :orientation]

  use BB.Message,
    schema: [
      position: [type: vec3_type(), required: true, doc: "Position in metres"],
      orientation: [type: quaternion_type(), required: true, doc: "Orientation as quaternion"]
    ]

  @type t :: %__MODULE__{
          position: Vec3.t(),
          orientation: Quaternion.t()
        }

  @doc """
  Create a new Pose message.

  Returns `{:ok, %BB.Message{}}` with the pose as payload.

  ## Examples

      alias BB.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:base_link, Vec3.new(1.0, 2.0, 3.0), Quaternion.identity())
  """
  @spec new(atom(), Vec3.t(), Quaternion.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = position, %Quaternion{} = orientation) do
    new(frame_id, position: position, orientation: orientation)
  end
end
