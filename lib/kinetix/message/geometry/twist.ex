# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Message.Geometry.Twist do
  @moduledoc """
  Linear and angular velocity in 3D space.

  ## Fields

  - `linear` - Linear velocity as `{:vec3, x, y, z}` in m/s
  - `angular` - Angular velocity as `{:vec3, x, y, z}` in rad/s

  ## Examples

      alias Kinetix.Message.Geometry.Twist
      alias Kinetix.Message.Vec3

      {:ok, msg} = Twist.new(:base_link, Vec3.new(1.0, 0.0, 0.0), Vec3.zero())
  """

  @behaviour Kinetix.Message

  import Kinetix.Message.Option

  defstruct [:linear, :angular]

  @type t :: %__MODULE__{
          linear: Kinetix.Message.Vec3.t(),
          angular: Kinetix.Message.Vec3.t()
        }

  @schema Spark.Options.new!(
            linear: [type: vec3_type(), required: true, doc: "Linear velocity in m/s"],
            angular: [type: vec3_type(), required: true, doc: "Angular velocity in rad/s"]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Twist message.

  Returns `{:ok, %Kinetix.Message{}}` with the twist as payload.

  ## Examples

      alias Kinetix.Message.Vec3

      {:ok, msg} = Twist.new(:base_link, Vec3.new(1.0, 0.0, 0.0), Vec3.zero())
  """
  @spec new(atom(), Kinetix.Message.Vec3.t(), Kinetix.Message.Vec3.t()) ::
          {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = linear, {:vec3, _, _, _} = angular) do
    Kinetix.Message.new(__MODULE__, frame_id,
      linear: linear,
      angular: angular
    )
  end
end
