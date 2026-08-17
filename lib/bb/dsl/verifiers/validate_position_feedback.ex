# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.Verifiers.ValidatePositionFeedback do
  @moduledoc """
  Warns about joints that are driven but never measured.

  `BB.Robot.State` is written from `BB.Message.Sensor.JointState` messages and
  from nothing else - commanding a joint doesn't move it in state, because a
  commanded position isn't a measured one. A joint with an actuator and no
  sensor therefore stays at its initial configuration forever, which quietly
  ruins anything reading it: forward kinematics, the URDF-driven visualisers,
  and inverse kinematics, which seeds each solve from the current
  configuration.

  Hardware without position feedback is served by
  `BB.Sensor.OpenLoopPositionEstimator`, which interpolates from the actuator's
  `BeginMotion` messages. Simulation adds one to every unsensed actuator
  automatically; hardware doesn't, since only the author knows whether the real
  device reports its own position.

  Which is the case this can't see: a smart servo that answers position queries
  on its bus is its own sensor, and the driver may publish `JointState` without
  anything appearing in the topology. `sensor false` on the actuator says so
  and silences the warning.

  This warns rather than failing the build. A robot that is only ever driven
  open-loop, never asked where it is, is unusual but not wrong.
  """

  use Spark.Dsl.Verifier

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

  defp unsensed_joints_in(%Link{} = link) do
    unsensed_joints(link.joints)
  end

  defp unsensed_joints_in(%Joint{type: :fixed}), do: []

  defp unsensed_joints_in(%Joint{sensors: [_ | _]} = joint), do: nested_joints(joint)

  defp unsensed_joints_in(%Joint{} = joint) do
    case Enum.filter(joint.actuators, & &1.sensor) do
      [] -> nested_joints(joint)
      actuators -> [{joint, actuators} | nested_joints(joint)]
    end
  end

  defp unsensed_joints_in(_entity), do: []

  defp nested_joints(%Joint{link: nil}), do: []
  defp nested_joints(%Joint{link: link}), do: unsensed_joints_in(link)

  defp warn({joint, actuators}, robot, file) do
    names = Enum.map_join(actuators, ", ", &inspect(&1.name))
    example = actuators |> List.first() |> Map.fetch!(:name)

    IO.warn(
      """
      #{inspect(robot)}: joint #{inspect(joint.name)} is driven by #{names} but has no sensor, \
      so nothing ever reports where it is. Its configuration in `BB.Robot.State` will stay at \
      its initial value, and inverse kinematics will keep solving from there.

      If the hardware has no position feedback, estimate it from the commands:

          sensor :#{example}_position, {BB.Sensor.OpenLoopPositionEstimator, actuator: :#{example}}

      If the actuator reports its own position, say so:

          actuator :#{example}, ..., sensor: false
      """,
      file: file,
      line: 1
    )
  end
end
