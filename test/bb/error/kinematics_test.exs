# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.KinematicsTest do
  use ExUnit.Case, async: true

  alias BB.Error.Kinematics.MultiFailed
  alias BB.Error.Kinematics.NoParentJoint
  alias BB.Error.Kinematics.NotAnAncestor
  alias BB.Error.Kinematics.UnknownActuator
  alias BB.Error.Kinematics.UnknownJoint
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Error.Kinematics.Unreachable

  describe "UnknownLink" do
    test "carries link, role and robot" do
      err = UnknownLink.exception(link: :gripper, role: :target, robot: MyRobot)

      assert err.link == :gripper
      assert err.role == :target
      assert err.robot == MyRobot
    end

    test "is an error" do
      assert BB.Error.severity(UnknownLink.exception(link: :gripper)) == :error
    end

    test "names the role so an unknown source isn't reported as a target" do
      source = UnknownLink.exception(link: :body, role: :source)
      target = UnknownLink.exception(link: :body, role: :target)

      assert UnknownLink.message(source) =~ "source link"
      assert UnknownLink.message(target) =~ "target link"
    end

    test "omits the role when a lookup takes only one link" do
      message = UnknownLink.message(UnknownLink.exception(link: :gripper))

      assert message =~ "Unknown link: :gripper"
      refute message =~ "source"
      refute message =~ "target"
    end

    test "message includes the robot when known" do
      err = UnknownLink.exception(link: :gripper, robot: MyRobot)
      assert UnknownLink.message(err) =~ "MyRobot"
    end
  end

  describe "UnknownJoint" do
    test "carries joint and robot" do
      err = UnknownJoint.exception(joint: :elbow, robot: MyRobot)

      assert err.joint == :elbow
      assert err.robot == MyRobot
    end

    test "is an error" do
      assert BB.Error.severity(UnknownJoint.exception(joint: :elbow)) == :error
    end

    test "message names the joint" do
      assert UnknownJoint.message(UnknownJoint.exception(joint: :elbow)) =~
               "Unknown joint: :elbow"
    end
  end

  describe "UnknownActuator" do
    test "carries actuator and robot" do
      err = UnknownActuator.exception(actuator: :pan_servo, robot: MyRobot)

      assert err.actuator == :pan_servo
      assert err.robot == MyRobot
    end

    test "is an error" do
      assert BB.Error.severity(UnknownActuator.exception(actuator: :pan_servo)) == :error
    end

    test "message names the actuator" do
      err = UnknownActuator.exception(actuator: :pan_servo)
      assert UnknownActuator.message(err) =~ "Unknown actuator: :pan_servo"
    end
  end

  describe "NoParentJoint" do
    test "carries the link" do
      assert NoParentJoint.exception(link: :world).link == :world
    end

    test "is an error" do
      assert BB.Error.severity(NoParentJoint.exception(link: :world)) == :error
    end

    test "message says the link is the root rather than that it doesn't exist" do
      message = NoParentJoint.message(NoParentJoint.exception(link: :world))

      assert message =~ ":world"
      assert message =~ "root"
      refute message =~ "not found"
    end
  end

  describe "NotAnAncestor" do
    test "carries both links and the common ancestor" do
      err =
        NotAnAncestor.exception(
          source_link: :left_gripper,
          target_link: :right_gripper,
          common_ancestor: :torso
        )

      assert err.source_link == :left_gripper
      assert err.target_link == :right_gripper
      assert err.common_ancestor == :torso
    end

    test "is an error" do
      err =
        NotAnAncestor.exception(
          source_link: :a,
          target_link: :b,
          common_ancestor: :root
        )

      assert BB.Error.severity(err) == :error
    end

    # The point of carrying the ancestor is that the message names the link the
    # caller should have passed, rather than just reporting a failure.
    test "message names the link the caller should have passed" do
      err =
        NotAnAncestor.exception(
          source_link: :left_gripper,
          target_link: :right_gripper,
          common_ancestor: :torso
        )

      message = NotAnAncestor.message(err)

      assert message =~ ":left_gripper is not an ancestor of :right_gripper"
      assert message =~ "nearest common ancestor is :torso"
    end
  end

  describe "MultiFailed" do
    # `partial_results` carries the failed target's own error alongside the ones
    # that solved, so a count of the whole map reads one too high — and it's a
    # user-visible number now that this error is what `move_to_multi/3` returns.
    test "message counts the targets that solved, not the entry that failed" do
      err =
        MultiFailed.exception(
          failed_link: :right_foot,
          error: Unreachable.exception(target_link: :right_foot),
          partial_results: %{
            left_foot: {:ok, %{hip: 0.5}, %{iterations: 3}},
            right_foot: {:error, Unreachable.exception(target_link: :right_foot)}
          }
        )

      assert MultiFailed.message(err) =~ "1 target(s) solved before failure"
    end

    test "and says none when the first target it tried was the one that failed" do
      err =
        MultiFailed.exception(
          failed_link: :right_foot,
          error: Unreachable.exception(target_link: :right_foot),
          partial_results: %{
            right_foot: {:error, Unreachable.exception(target_link: :right_foot)}
          }
        )

      assert MultiFailed.message(err) =~ "0 target(s) solved before failure"
    end
  end
end
