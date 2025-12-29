# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Accel do
  @moduledoc """
  Linear and angular acceleration in 3D space.

  ## Fields

  - `linear` - Linear acceleration as `BB.Vec3.t()` in m/s²
  - `angular` - Angular acceleration as `BB.Vec3.t()` in rad/s²

  ## Examples

      alias BB.Message.Geometry.Accel
      alias BB.Math.Vec3

      {:ok, msg} = Accel.new(:base_link, Vec3.new(0.0, 0.0, 9.81), Vec3.zero())
  """

  import BB.Message.Option

  alias BB.Math.Vec3

  defstruct [:linear, :angular]

  use BB.Message,
    schema: [
      linear: [type: vec3_type(), required: true, doc: "Linear acceleration in m/s²"],
      angular: [type: vec3_type(), required: true, doc: "Angular acceleration in rad/s²"]
    ]

  @type t :: %__MODULE__{
          linear: Vec3.t(),
          angular: Vec3.t()
        }

  @doc """
  Create a new Accel message.

  Returns `{:ok, %BB.Message{}}` with the acceleration as payload.

  ## Examples

      alias BB.Math.Vec3

      {:ok, msg} = Accel.new(:base_link, Vec3.new(0.0, 0.0, 9.81), Vec3.zero())
  """
  @spec new(atom(), Vec3.t(), Vec3.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = linear, %Vec3{} = angular) do
    new(frame_id, linear: linear, angular: angular)
  end
end
