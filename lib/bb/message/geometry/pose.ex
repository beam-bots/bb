# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Pose do
  @moduledoc """
  A position and orientation in 3D space.

  Wraps a `BB.Math.Transform` for use as a message payload.

  ## Fields

  - `transform` - The pose as `BB.Math.Transform.t()`

  ## Examples

      alias BB.Message.Geometry.Pose
      alias BB.Math.{Vec3, Quaternion, Transform}

      # Create from Transform
      transform = Transform.from_position_quaternion(Vec3.new(1.0, 0.0, 0.5), Quaternion.identity())
      {:ok, msg} = Pose.new(:end_effector, transform)

      # Or from position and orientation
      {:ok, msg} = Pose.new(:end_effector, Vec3.new(1.0, 0.0, 0.5), Quaternion.identity())

      # Access components
      pose = msg.payload
      Pose.position(pose)     # => %Vec3{}
      Pose.orientation(pose)  # => %Quaternion{}
      Pose.to_transform(pose) # => %Transform{}
  """

  import BB.Message.Option

  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3

  defstruct [:transform]

  use BB.Message,
    schema: [
      transform: [type: transform_type(), required: true, doc: "Pose as Transform"]
    ]

  @type t :: %__MODULE__{transform: Transform.t()}

  @doc """
  Create a new Pose message from a Transform.

  ## Examples

      alias BB.Math.Transform

      {:ok, msg} = Pose.new(:base_link, Transform.identity())
  """
  @spec new(atom(), Transform.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Transform{} = transform) do
    new(frame_id, transform: transform)
  end

  @doc """
  Create a new Pose message from position and orientation.

  ## Examples

      alias BB.Math.{Vec3, Quaternion}

      {:ok, msg} = Pose.new(:base_link, Vec3.new(1.0, 2.0, 3.0), Quaternion.identity())
  """
  @spec new(atom(), Vec3.t(), Quaternion.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = position, %Quaternion{} = orientation) do
    transform = Transform.from_position_quaternion(position, orientation)
    new(frame_id, transform: transform)
  end

  @doc "Get the position component as Vec3."
  @spec position(t()) :: Vec3.t()
  def position(%__MODULE__{transform: transform}), do: Transform.get_translation(transform)

  @doc "Get the orientation component as Quaternion."
  @spec orientation(t()) :: Quaternion.t()
  def orientation(%__MODULE__{transform: transform}), do: Transform.get_quaternion(transform)

  @doc "Get the underlying Transform."
  @spec to_transform(t()) :: Transform.t()
  def to_transform(%__MODULE__{transform: transform}), do: transform
end
