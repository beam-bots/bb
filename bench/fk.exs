# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

# Forward-kinematics baseline on the 6-DOF example arm. `forward_kinematics/3`
# walks a single chain to one link; `all_link_transforms/2` computes every link
# pose. Both currently run as eager per-joint `Transform.compose` calls — this
# measures the cost #147 aims to cut by expressing the chain walk as a single
# `defn`.
#
# Run with: mix run bench/fk.exs
# (BB.ExampleRobots is compiled from test/support in dev and test envs.)

alias BB.ExampleRobots.SixDofArm
alias BB.Robot.Kinematics

robot = SixDofArm.robot()

positions = %{
  shoulder_pan_joint: 0.3,
  shoulder_lift_joint: -0.5,
  elbow_joint: 0.8,
  wrist_1_joint: -0.4,
  wrist_2_joint: 0.6,
  wrist_3_joint: 0.2
}

Benchee.run(
  %{
    "forward_kinematics/3 (base -> tool0)" => fn ->
      Kinematics.forward_kinematics(robot, positions, :tool0)
    end,
    "all_link_transforms/2 (all links)" => fn ->
      Kinematics.all_link_transforms(robot, positions)
    end
  },
  warmup: 1,
  time: 3
)
