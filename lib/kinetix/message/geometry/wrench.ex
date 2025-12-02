defmodule Kinetix.Message.Geometry.Wrench do
  @moduledoc """
  Force and torque in 3D space.

  ## Fields

  - `force` - Force as `{:vec3, x, y, z}` in Newtons
  - `torque` - Torque as `{:vec3, x, y, z}` in Newton-metres

  ## Examples

      alias Kinetix.Message.Geometry.Wrench
      alias Kinetix.Message.Vec3

      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0.0, 0.0, -10.0), Vec3.zero())
  """

  @behaviour Kinetix.Message

  import Kinetix.Message.Option

  defstruct [:force, :torque]

  @type t :: %__MODULE__{
          force: Kinetix.Message.Vec3.t(),
          torque: Kinetix.Message.Vec3.t()
        }

  @schema Spark.Options.new!(
            force: [type: vec3_type(), required: true, doc: "Force in Newtons"],
            torque: [type: vec3_type(), required: true, doc: "Torque in Newton-metres"]
          )

  @impl Kinetix.Message
  def schema, do: @schema

  defimpl Kinetix.Message.Payload do
    def schema(_), do: @for.schema()
  end

  @doc """
  Create a new Wrench message.

  Returns `{:ok, %Kinetix.Message{}}` with the wrench as payload.

  ## Examples

      alias Kinetix.Message.Vec3

      {:ok, msg} = Wrench.new(:end_effector, Vec3.new(0.0, 0.0, -10.0), Vec3.zero())
  """
  @spec new(atom(), Kinetix.Message.Vec3.t(), Kinetix.Message.Vec3.t()) ::
          {:ok, Kinetix.Message.t()} | {:error, term()}
  def new(frame_id, {:vec3, _, _, _} = force, {:vec3, _, _, _} = torque) do
    Kinetix.Message.new(__MODULE__, frame_id,
      force: force,
      torque: torque
    )
  end
end
