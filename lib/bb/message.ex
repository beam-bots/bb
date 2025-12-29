# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message do
  @moduledoc """
  Message envelope and behaviour for payload types.

  Messages in BB are wrapped in a standard envelope containing timing,
  coordinate frame, and payload data.

  ## Usage

  Use the `use BB.Message` macro to define a payload type:

      defmodule MyPayload do
        defstruct [:value]

        use BB.Message,
          schema: [
            value: [type: :float, required: true]
          ]
      end

      {:ok, msg} = MyPayload.new(:base_link, value: 1.5)

  The macro will:
  - Validate struct fields match schema keys at compile time
  - Implement the `schema/0` callback
  - Generate a `new/2` helper function

  Note: `defstruct` must be defined before `use BB.Message`.
  """

  defstruct [:timestamp, :frame_id, :payload]

  @type t :: %__MODULE__{
          timestamp: integer(),
          frame_id: atom(),
          payload: struct()
        }

  @doc "Returns a compiled Spark.Options schema for this payload type"
  @callback schema() :: Spark.Options.t()

  @doc false
  defmacro __using__(opts) do
    schema_opts = opts[:schema] || raise ArgumentError, "schema option is required"

    quote do
      @behaviour BB.Message

      @__bb_message_schema Spark.Options.new!(unquote(schema_opts))

      @impl BB.Message
      def schema, do: @__bb_message_schema

      @spec new(atom(), keyword()) :: {:ok, BB.Message.t()} | {:error, term()}
      def new(frame_id, attrs) when is_atom(frame_id) and is_list(attrs) do
        BB.Message.new(__MODULE__, frame_id, attrs)
      end
    end
  end

  @doc """
  Create a new message with validated payload.

  Validates the attributes against the payload module's schema, then wraps
  the resulting struct in a message envelope with a fresh timestamp.

  ## Examples

      alias BB.Message.Geometry.Pose
      alias BB.Math.Transform

      {:ok, msg} = BB.Message.new(Pose, :end_effector, [
        transform: Transform.identity()
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
        transform: Transform.identity()
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

  ## Examples

      BB.Message.schema(msg)  #=> %Spark.Options{...}
  """
  @spec schema(t()) :: Spark.Options.t()
  def schema(%__MODULE__{payload: payload}), do: payload.__struct__.schema()
end
