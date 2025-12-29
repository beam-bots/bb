# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Point3D do
  @moduledoc """
  A 3D point in space.

  Wraps a `BB.Math.Vec3` for use as a message payload.

  ## Fields

  - `vec` - The point as `BB.Math.Vec3.t()` in metres

  ## Examples

      alias BB.Message.Geometry.Point3D
      alias BB.Math.Vec3

      {:ok, msg} = Point3D.new(:base_link, Vec3.new(0.3, 0.2, 0.1))

      # Access coordinates
      point = msg.payload
      Point3D.x(point)  # => 0.3
      Point3D.to_vec3(point)  # => %Vec3{}
  """

  import BB.Message.Option

  alias BB.Math.Vec3

  defstruct [:vec]

  use BB.Message,
    schema: [
      vec: [type: vec3_type(), required: true, doc: "Point as Vec3 in metres"]
    ]

  @type t :: %__MODULE__{vec: Vec3.t()}

  @doc """
  Create a new Point3D message from a Vec3.

  ## Examples

      alias BB.Math.Vec3

      {:ok, msg} = Point3D.new(:base_link, Vec3.new(0.3, 0.2, 0.1))
  """
  @spec new(atom(), Vec3.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = vec) do
    new(frame_id, vec: vec)
  end

  @doc "Get the X coordinate."
  @spec x(t()) :: float()
  def x(%__MODULE__{vec: vec}), do: Vec3.x(vec)

  @doc "Get the Y coordinate."
  @spec y(t()) :: float()
  def y(%__MODULE__{vec: vec}), do: Vec3.y(vec)

  @doc "Get the Z coordinate."
  @spec z(t()) :: float()
  def z(%__MODULE__{vec: vec}), do: Vec3.z(vec)

  @doc "Get the underlying Vec3."
  @spec to_vec3(t()) :: Vec3.t()
  def to_vec3(%__MODULE__{vec: vec}), do: vec
end
