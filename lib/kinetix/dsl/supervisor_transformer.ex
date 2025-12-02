defmodule Kinetix.Dsl.SupervisorTransformer do
  @moduledoc """
  Injects `start_link/1` and `child_spec/1` into robot modules.

  This allows robot modules to be started directly with `MyRobot.start_link()`
  and to be used as child specs in supervision trees.
  """
  use Spark.Dsl.Transformer
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(_), do: true

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @doc """
         Returns a child specification for starting this robot under a supervisor.
         """
         @spec child_spec(Keyword.t()) :: Supervisor.child_spec()
         def child_spec(opts \\ []) do
           %{
             id: unquote(module),
             start: {unquote(module), :start_link, [opts]},
             type: :supervisor
           }
         end

         @doc """
         Starts the robot's supervision tree.

         ## Options

         All options are passed through to sensor and actuator child processes.
         """
         @spec start_link(Keyword.t()) :: Supervisor.on_start()
         def start_link(opts \\ []) do
           Kinetix.Supervisor.start_link(unquote(module), opts)
         end
       end
     )}
  end
end
