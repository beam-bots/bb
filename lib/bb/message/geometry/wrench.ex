# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Wrench do
  @moduledoc """
  Force and torque in 3D space.

  ## Fields

  - `force` - Force as `{:vec3, x, y, z}` in Newtons
  - `torque` - Torque as `{:vec3, x, y, z}` in Newton-metres

  ## Examples

      alias BB.Message.Geometry.Wrench
      alias BB.Message.Vec3

      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0.0, 0.0, -10.0), Vec3.zero())
  """

  @behaviour BB.Message

  import BB.Message.Option

  defstruct [:force, :torque]

  @type t :: %__MODULE__{
          force: BB.Message.Vec3.t(),
          torque: BB.Message.Vec3.t()
        }

  @schema Spark.Options.new!(
            force: [type: vec3_type(), required: true, doc: "Force in Newtons"],
            torque: [type: vec3_type(), required: true, doc: "Torque in Newton-metres"]
          )

  @impl BB.Message
  def schema, do: @schema

  defimpl BB.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Wrench message.

  Returns `{:ok, %BB.Message{}}` with the wrench as payload.

  ## Examples

      alias BB.Message.Vec3

      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0.0, 0.0, -10.0), Vec3.zero())
  """
  @spec new(atom(), BB.Message.Vec3.t(), BB.Message.Vec3.t()) ::
          {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = force, {:vec3, _, _, _} = torque) do
    BB.Message.new(__MODULE__, frame_id,
      force: force,
      torque: torque
    )
  end
end
