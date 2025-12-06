# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message do
  @moduledoc """
  Message envelope and behaviour for payload types.

  Messages in BB are wrapped in a standard envelope containing timing,
  coordinate frame, and payload data.

  ## Behaviour

  Payload modules must implement the `BB.Message` behaviour:

  - `schema/0` - Returns a compiled `Spark.Options` schema for validation

  ## Protocol

  Payload structs must also implement the `BB.Message.Payload` protocol
  for runtime introspection.

  ## Example

      defmodule MyPayload do
        @behaviour BB.Message

        defstruct [:value]

        @schema Spark.Options.new!([
          value: [type: :float, required: true]
        ])

        @impl BB.Message
        def schema, do: @schema

        defimpl BB.Message.Payload do
          def schema(_), do: @for.schema()
        end
      end

      {:ok, msg} = BB.Message.new(MyPayload, :base_link, value: 1.5)
  """

  alias BB.Message.Payload

  defstruct [:timestamp, :frame_id, :payload]

  @type t :: %__MODULE__{
          timestamp: integer(),
          frame_id: atom(),
          payload: struct()
        }

  @doc "Returns a compiled Spark.Options schema for this payload type"
  @callback schema() :: Spark.Options.t()

  @doc """
  Create a new message with validated payload.

  Validates the attributes against the payload module's schema, then wraps
  the resulting struct in a message envelope with a fresh timestamp.

  ## Examples

      alias BB.Message.Geometry.Pose
      alias BB.Message.{Vec3, Quaternion}

      {:ok, msg} = BB.Message.new(Pose, :end_effector, [
        position: Vec3.new(1.0, 0.0, 0.5),
        orientation: Quaternion.identity()
      ])
  """
  @spec new(module(), atom(), keyword()) :: {:ok, t()} | {:error, term()}
  def new(payload_module, frame_id, attrs)
      when is_atom(payload_module) and is_atom(frame_id) and is_list(attrs) do
    case Spark.Options.validate(attrs, payload_module.schema()) do
      {:ok, validated} ->
        {:ok,
         %__MODULE__{
           timestamp: System.monotonic_time(:nanosecond),
           frame_id: frame_id,
           payload: struct(payload_module, validated)
         }}

      {:error, _} = err ->
        err
    end
  end

  @doc """
  Like `new/3` but raises on validation error.

  ## Examples

      msg = BB.Message.new!(Pose, :end_effector, [
        position: Vec3.new(1.0, 0.0, 0.5),
        orientation: Quaternion.identity()
      ])
  """
  @spec new!(module(), atom(), keyword()) :: t()
  def new!(payload_module, frame_id, attrs) do
    case new(payload_module, frame_id, attrs) do
      {:ok, msg} -> msg
      {:error, err} -> raise err
    end
  end

  @doc """
  Get the payload schema from a message.

  Delegates to the `BB.Message.Payload` protocol.

  ## Examples

      BB.Message.schema(msg)  #=> %Spark.Options{...}
  """
  @spec schema(t()) :: Spark.Options.t()
  def schema(%__MODULE__{payload: payload}), do: Payload.schema(payload)
end
