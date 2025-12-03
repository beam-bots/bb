# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Message.Sensor.Imu do
  @moduledoc """
  Inertial Measurement Unit data.

  ## Fields

  - `orientation` - Orientation as `{:quaternion, x, y, z, w}`
  - `angular_velocity` - Angular velocity as `{:vec3, x, y, z}` in rad/s
  - `linear_acceleration` - Linear acceleration as `{:vec3, x, y, z}` in m/s²

  ## Examples

      alias Kinetix.Message.Sensor.Imu
      alias Kinetix.Message.{Vec3, Quaternion}

      {:ok, msg} = Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
      )
  """

  @behaviour Kinetix.Message

  import Kinetix.Message.Option

  defstruct [:orientation, :angular_velocity, :linear_acceleration]

  @type t :: %__MODULE__{
          orientation: Kinetix.Message.Quaternion.t(),
          angular_velocity: Kinetix.Message.Vec3.t(),
          linear_acceleration: Kinetix.Message.Vec3.t()
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

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Imu message.

  Returns `{:ok, %Kinetix.Message{}}` with the IMU data as payload.

  ## Examples

      alias Kinetix.Message.{Vec3, Quaternion}

      {:ok, msg} = Imu.new(:imu_link,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
      )
  """
  @spec new(atom(), keyword()) :: {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
    Kinetix.Message.new(__MODULE__, frame_id, attrs)
  end
end
