# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Twist do
  @moduledoc """
  Linear and angular velocity in 3D space.

  ## Fields

  - `linear` - Linear velocity as `BB.Vec3.t()` in m/s
  - `angular` - Angular velocity as `BB.Vec3.t()` in rad/s

  ## Examples

      alias BB.Message.Geometry.Twist
      alias BB.Math.Vec3

      {:ok, msg} = Twist.new(:base_link, Vec3.new(1.0, 0.0, 0.0), Vec3.zero())
  """

  import BB.Message.Option

  alias BB.Math.Vec3

  defstruct [:linear, :angular]

  use BB.Message,
    schema: [
      linear: [type: vec3_type(), required: true, doc: "Linear velocity in m/s"],
      angular: [type: vec3_type(), required: true, doc: "Angular velocity in rad/s"]
    ]

  @type t :: %__MODULE__{
          linear: Vec3.t(),
          angular: Vec3.t()
        }

  @doc """
  Create a new Twist message.

  Returns `{:ok, %BB.Message{}}` with the twist as payload.

  ## Examples

      alias BB.Math.Vec3

      {:ok, msg} = Twist.new(:base_link, Vec3.new(1.0, 0.0, 0.0), Vec3.zero())
  """
  @spec new(atom(), Vec3.t(), Vec3.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = linear, %Vec3{} = angular) do
    new(frame_id, linear: linear, angular: angular)
  end
end
