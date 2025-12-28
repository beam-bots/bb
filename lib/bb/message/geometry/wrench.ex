# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Wrench do
  @moduledoc """
  Force and torque in 3D space.

  ## Fields

  - `force` - Force as `BB.Vec3.t()` in Newtons
  - `torque` - Torque as `BB.Vec3.t()` in Newton-metres

  ## Examples

      alias BB.Message.Geometry.Wrench
      alias BB.Math.Vec3

      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0.0, 0.0, -10.0), Vec3.zero())
  """

  import BB.Message.Option

  alias BB.Math.Vec3

  defstruct [:force, :torque]

  use BB.Message,
    schema: [
      force: [type: vec3_type(), required: true, doc: "Force in Newtons"],
      torque: [type: vec3_type(), required: true, doc: "Torque in Newton-metres"]
    ]

  @type t :: %__MODULE__{
          force: Vec3.t(),
          torque: Vec3.t()
        }

  @doc """
  Create a new Wrench message.

  Returns `{:ok, %BB.Message{}}` with the wrench as payload.

  ## Examples

      alias BB.Math.Vec3

      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0.0, 0.0, -10.0), Vec3.zero())
  """
  @spec new(atom(), Vec3.t(), Vec3.t()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, %Vec3{} = force, %Vec3{} = torque) do
    new(frame_id, force: force, torque: torque)
  end
end
