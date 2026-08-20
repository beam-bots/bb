# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidatePositionFeedback do
  @moduledoc """
  Warns about joints that are driven but never measured.

  `BB.Robot.State` is written from `BB.Message.Sensor.JointState` messages and
  from nothing else - commanding a joint doesn't move it in state, because a
  commanded position isn't a measured one. A joint that nothing reports on
  therefore stays at its initial configuration forever, which quietly ruins
  anything reading it: forward kinematics, the URDF-driven visualisers, and
  inverse kinematics, which seeds each solve from the current configuration.

  Two things can report on a joint. A sensor declared alongside the actuator -
  an encoder, or `BB.Sensor.OpenLoopPositionEstimator` interpolating from the
  actuator's `BeginMotion` messages for hardware with no feedback at all. Or
  the actuator itself, if it reads position back from the hardware and says so
  through `c:BB.Actuator.capabilities/1`. A joint with neither gets a warning
  naming both fixes.

  The driver is asked directly rather than the robot's author being made to
  declare it, because whether a smart servo answers position queries is a
  property of the driver, not of the robot it's wired into.

  Simulation adds an estimator to every actuator without a declared position
  sensor; hardware doesn't, which is why this check exists.

  It warns rather than failing the build. A robot that is only ever driven
  open-loop, never asked where it is, is unusual but not wrong.
  """

  use Spark.Dsl.Verifier

  alias BB.Dsl.ChildSpecOptions
  alias BB.Dsl.Joint
  alias BB.Dsl.Link
  alias Spark.Dsl.Verifier

  @impl true
  def verify(dsl_state) do
    robot = Verifier.get_persisted(dsl_state, :module)
    file = Verifier.get_persisted(dsl_state, :file)

    dsl_state
    |> Verifier.get_entities([:topology])
    |> unsensed_joints()
    |> Enum.each(&warn(&1, robot, file))

    :ok
  end

  defp unsensed_joints(entities) do
    Enum.flat_map(entities, &unsensed_joints_in/1)
  end

  defp unsensed_joints_in(%Link{} = link), do: unsensed_joints(link.joints)

  # A fixed joint has nowhere to go. A continuous one rotates without limit - a
  # wheel, a spinner - so it has no position to hold, nothing solves toward it,
  # and the estimator this warning recommends can't help: it interpolates
  # position moves, which a velocity- or effort-driven wheel never issues.
  defp unsensed_joints_in(%Joint{type: type}) when type in [:fixed, :continuous], do: []

  defp unsensed_joints_in(%Joint{sensors: [_ | _]} = joint), do: nested_joints(joint)

  defp unsensed_joints_in(%Joint{} = joint) do
    case Enum.reject(joint.actuators, &senses_position?/1) do
      [] -> nested_joints(joint)
      actuators -> [{joint, actuators} | nested_joints(joint)]
    end
  end

  defp unsensed_joints_in(_entity), do: []

  defp nested_joints(%Joint{link: nil}), do: []
  defp nested_joints(%Joint{link: link}), do: unsensed_joints_in(link)

  # A driver BB can't load can't be asked, and refusing to compile over it would
  # break the cross-package builds `ValidateChildSpecBehavioursTransformer`
  # already goes out of its way to allow. Say nothing rather than guess.
  defp senses_position?(actuator) do
    {module, opts} = ChildSpecOptions.module_and_options(actuator.child_spec)

    case Code.ensure_compiled(module) do
      {:module, _} -> :position_feedback in capabilities(module, opts)
      {:error, _reason} -> true
    end
  end

  # Options the driver couldn't have asked for are worse than none: invalid ones
  # are `ValidateChildSpecs`' to report, and this verifier would only turn its
  # error into a crash.
  defp capabilities(module, opts) do
    with true <- function_exported?(module, :capabilities, 1),
         {:ok, opts} <- ChildSpecOptions.validate(module, opts) do
      module.capabilities(opts)
    else
      _ -> []
    end
  end

  # Sorted because the DSL hands entities back in whatever order it holds them,
  # and a warning that reorders itself between builds is a warning people learn
  # to distrust.
  defp warn({joint, actuators}, robot, file) do
    [example | _] = sorted = actuators |> Enum.map(& &1.name) |> Enum.sort()
    names = Enum.map_join(sorted, ", ", &inspect/1)

    IO.warn(
      """
      #{inspect(robot)}: joint #{inspect(joint.name)} is driven by #{names} but nothing reports \
      where it is, so its configuration in `BB.Robot.State` will stay at its initial value and \
      inverse kinematics will keep solving from there.

      If the hardware has no position feedback, estimate it from the commands:

          sensor :#{example}_position, {BB.Sensor.OpenLoopPositionEstimator, actuator: :#{example}}

      If the driver does read position back from the hardware, it should say so and publish it as \
      `BB.Message.Sensor.JointState`:

          @impl BB.Actuator
          def capabilities(_opts), do: [:position_feedback]
      """,
      file: file,
      line: 1
    )
  end
end
