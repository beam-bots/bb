# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl.RobotTransformer do
  @moduledoc """
  Builds and persists the optimised Robot struct at compile-time.

  This transformer runs after the LinkTransformer and UniquenessTransformer to
  ensure the DSL is fully validated, then builds the optimised `Kinetix.Robot`
  struct and injects an accessor function into the robot module.
  """
  use Spark.Dsl.Transformer
  alias Kinetix.Dsl.Link
  alias Kinetix.Robot.Builder
  alias Spark.Dsl.Transformer

  @doc false
  @impl true
  def after?(Kinetix.Dsl.LinkTransformer), do: true
  def after?(Kinetix.Dsl.SupervisorTransformer), do: true
  def after?(Kinetix.Dsl.UniquenessTransformer), do: true
  def after?(_), do: false

  @doc false
  @impl true
  def before?(_), do: false

  @doc false
  @impl true
  def transform(dsl) do
    module = Transformer.get_persisted(dsl, :module)

    dsl
    |> Transformer.get_entities([:robot])
    |> Enum.filter(&is_struct(&1, Link))
    |> case do
      [] ->
        {:ok, dsl}

      [root_link] ->
        robot = Builder.build_from_dsl(module, root_link)
        inject_robot_accessor(dsl, module, robot)
    end
  end

  defp inject_robot_accessor(dsl, module, robot) do
    robot_data = Macro.escape(robot)

    {:ok,
     Transformer.eval(
       dsl,
       [],
       quote do
         @kinetix_robot unquote(robot_data)

         @doc """
         Returns the optimised robot representation.

         This struct is built at compile-time from the DSL definition and contains:
         - All physical values converted to SI base units (floats)
         - Flat maps for O(1) lookup of links, joints, sensors, and actuators
         - Pre-computed topology metadata for efficient traversal

         ## Examples

             robot = #{unquote(module)}.robot()
             link = Kinetix.Robot.get_link(robot, :base_link)
             joint = Kinetix.Robot.get_joint(robot, :shoulder)

         """
         @spec robot() :: Kinetix.Robot.t()
         def robot, do: @kinetix_robot
       end
     )}
  end
end
