# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.Imu do
  @moduledoc """
  Inertial Measurement Unit data.

  ## Fields

  - `orientation` - Orientation as `{:quaternion, x, y, z, w}`
  - `angular_velocity` - Angular velocity as `{:vec3, x, y, z}` in rad/s
  - `linear_acceleration` - Linear acceleration as `{:vec3, x, y, z}` in m/s²

  ## Examples

      alias BB.Message.Sensor.Imu
      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
      )
  """

  @behaviour BB.Message

  import BB.Message.Option

  defstruct [:orientation, :angular_velocity, :linear_acceleration]

  @type t :: %__MODULE__{
          orientation: BB.Message.Quaternion.t(),
          angular_velocity: BB.Message.Vec3.t(),
          linear_acceleration: BB.Message.Vec3.t()
        }

  @schema Spark.Options.new!(
            orientation: [
              type: quaternion_type(),
              required: true,
              doc: "Orientation as quaternion"
            ],
            angular_velocity: [
              type: vec3_type(),
              required: true,
              doc: "Angular velocity in rad/s"
            ],
            linear_acceleration: [
              type: vec3_type(),
              required: true,
              doc: "Linear acceleration in m/s²"
            ]
          )

  @impl BB.Message
  def schema, do: @schema

  defimpl BB.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Imu message.

  Returns `{:ok, %BB.Message{}}` with the IMU data as payload.

  ## Examples

      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
      )
  """
  @spec new(atom(), keyword()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    BB.Message.new(__MODULE__, frame_id, attrs)
  end
end
