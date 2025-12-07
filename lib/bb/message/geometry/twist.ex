# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Twist do
  @moduledoc """
  Linear and angular velocity in 3D space.

  ## Fields

  - `linear` - Linear velocity as `{:vec3, x, y, z}` in m/s
  - `angular` - Angular velocity as `{:vec3, x, y, z}` in rad/s

  ## Examples

      alias BB.Message.Geometry.Twist
      alias BB.Message.Vec3

      {:ok, msg} = Twist.new(:base_link, Vec3.new(1.0, 0.0, 0.0), Vec3.zero())
  """

  import BB.Message.Option

  defstruct [:linear, :angular]

  use BB.Message,
    schema: [
      linear: [type: vec3_type(), required: true, doc: "Linear velocity in m/s"],
      angular: [type: vec3_type(), required: true, doc: "Angular velocity in rad/s"]
    ]

  @type t :: %__MODULE__{
          linear: BB.Message.Vec3.t(),
          angular: BB.Message.Vec3.t()
        }

  @doc """
  Create a new Twist message.

  Returns `{:ok, %BB.Message{}}` with the twist as payload.

  ## Examples

      alias BB.Message.Vec3

      {:ok, msg} = Twist.new(:base_link, Vec3.new(1.0, 0.0, 0.0), Vec3.zero())
  """
  @spec new(atom(), BB.Message.Vec3.t(), BB.Message.Vec3.t()) ::
          {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = linear, {:vec3, _, _, _} = angular) do
    new(frame_id, linear: linear, angular: angular)
  end
end
