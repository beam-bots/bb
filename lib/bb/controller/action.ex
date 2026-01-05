# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Controller.Action do
  @moduledoc """
  Action builders and executor for reactive controllers.

  Provides two action types:
  - `Command` - invokes a robot command
  - `Callback` - calls an arbitrary function with the message and context

  ## DSL Builders

  These functions are imported into the controller entity scope:

      controller :over_current, {BB.Controller.Threshold,
        topic: [:sensor, :servo_status],
        field: :current,
        max: 1.21,
        action: command(:disarm)
      }

      controller :collision, {BB.Controller.PatternMatch,
        topic: [:sensor, :proximity],
        match: fn msg -> msg.payload.distance < 0.05 end,
        action: handle_event(fn msg, ctx ->
          Logger.warning("Collision detected")
          :ok
        end)
      }
  """

  alias BB.Controller.Action.{Callback, Command, Context}

  defmodule Command do
    @moduledoc "Action that invokes a robot command."
    defstruct [:command, args: []]

    @type t :: %__MODULE__{
            command: atom(),
            args: keyword()
          }
  end

  defmodule Callback do
    @moduledoc "Action that calls an arbitrary function."
    defstruct [:handler]

    @type t :: %__MODULE__{
            handler: (BB.Message.t(), Context.t() -> any())
          }
  end

  defmodule Context do
    @moduledoc """
    Context provided to action callbacks.

    Contains references to the robot module, static topology, dynamic state,
    and the controller name that triggered the action.
    """
    defstruct [:robot_module, :robot, :robot_state, :controller_name]

    @type t :: %__MODULE__{
            robot_module: module(),
            robot: BB.Robot.t(),
            robot_state: BB.Robot.Runtime.robot_state(),
            controller_name: atom()
          }
  end

  @type t :: Command.t() | Callback.t()

  @doc """
  Build a command action that invokes the named robot command.

  ## Examples

      command(:disarm)
      command(:move_to, target: pose)
  """
  @spec command(atom()) :: Command.t()
  def command(name) when is_atom(name), do: %Command{command: name}

  @spec command(atom(), keyword()) :: Command.t()
  def command(name, args) when is_atom(name) and is_list(args),
    do: %Command{command: name, args: args}

  @doc """
  Build a callback action that calls the given function.

  The function receives the triggering message and a context struct.

  ## Examples

      handle_event(fn msg, ctx ->
        Logger.info("Received: \#{inspect(msg)}")
        :ok
      end)
  """
  @spec handle_event((BB.Message.t(), Context.t() -> any())) :: Callback.t()
  def handle_event(fun) when is_function(fun, 2), do: %Callback{handler: fun}

  @doc """
  Execute an action with the given message and context.
  """
  @spec execute(t(), BB.Message.t(), Context.t()) :: any()
  def execute(%Command{command: cmd, args: args}, _message, %Context{robot_module: robot}) do
    apply(robot, cmd, [Map.new(args)])
  end

  def execute(%Callback{handler: fun}, message, %Context{} = context) do
    fun.(message, context)
  end
end
