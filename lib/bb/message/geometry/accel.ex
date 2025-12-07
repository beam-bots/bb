# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Accel do
  @moduledoc """
  Linear and angular acceleration in 3D space.

  ## Fields

  - `linear` - Linear acceleration as `{:vec3, x, y, z}` in m/s²
  - `angular` - Angular acceleration as `{:vec3, x, y, z}` in rad/s²

  ## Examples

      alias BB.Message.Geometry.Accel
      alias BB.Message.Vec3

      {:ok, msg} = Accel.new(:base_link, Vec3.new(0.0, 0.0, 9.81), Vec3.zero())
  """

  import BB.Message.Option

  defstruct [:linear, :angular]

  use BB.Message,
    schema: [
      linear: [type: vec3_type(), required: true, doc: "Linear acceleration in m/s²"],
      angular: [type: vec3_type(), required: true, doc: "Angular acceleration in rad/s²"]
    ]

  @type t :: %__MODULE__{
          linear: BB.Message.Vec3.t(),
          angular: BB.Message.Vec3.t()
        }

  @doc """
  Create a new Accel message.

  Returns `{:ok, %BB.Message{}}` with the acceleration as payload.

  ## Examples

      alias BB.Message.Vec3

      {:ok, msg} = Accel.new(:base_link, Vec3.new(0.0, 0.0, 9.81), Vec3.zero())
  """
  @spec new(atom(), BB.Message.Vec3.t(), BB.Message.Vec3.t()) ::
          {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = linear, {:vec3, _, _, _} = angular) do
    new(frame_id, linear: linear, angular: angular)
  end
end
