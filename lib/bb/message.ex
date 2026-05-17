# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message do
  @moduledoc """
  Message envelope and behaviour for payload types.

  Messages in BB are wrapped in a standard envelope containing timing,
  coordinate frame, and payload data.

  The `:robot` field identifies the publishing robot module and is filled
  in automatically by `BB.PubSub.publish/3` just before dispatch. It is
  `nil` on freshly-constructed messages that have not yet been published,
  and lets subscribers that listen to more than one robot attribute each
  delivered message to its source.

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

  defstruct [:monotonic_time, :wall_time, :node, :frame_id, :payload, :robot]

  @type t :: %__MODULE__{
          monotonic_time: integer(),
          wall_time: integer(),
          node: node(),
          frame_id: atom(),
          payload: struct(),
          robot: module() | nil
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
  the resulting struct in a message envelope. The envelope is stamped with
  the current monotonic time, wall-clock time, and the local node name,
  so messages remain interpretable across nodes and after recording.

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
           monotonic_time: System.monotonic_time(:nanosecond),
           wall_time: System.system_time(:nanosecond),
           node: Node.self(),
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
