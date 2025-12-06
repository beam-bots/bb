# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.ActuatorTest do
  use ExUnit.Case, async: true
  alias BB.Dsl.{Actuator, Info}

  describe "joint actuator with bare module" do
    defmodule BareModuleActuatorRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type :revolute

            actuator(:motor, ServoMotor)

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :arm do
            end
          end
        end
      end
    end

    test "actuator on joint with bare module" do
      [link] = Info.topology(BareModuleActuatorRobot)
      [joint] = link.joints
      [actuator] = joint.actuators
      assert is_struct(actuator, Actuator)
      assert actuator.name == :motor
      assert actuator.child_spec == ServoMotor
    end
  end

  describe "joint actuator with module and args" do
    defmodule ModuleArgsActuatorRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type :revolute

            actuator(:motor, {ServoMotor, pwm_pin: 12, frequency: 50})

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :arm do
            end
          end
        end
      end
    end

    test "actuator on joint with module and args" do
      [link] = Info.topology(ModuleArgsActuatorRobot)
      [joint] = link.joints
      [actuator] = joint.actuators
      assert actuator.name == :motor
      assert actuator.child_spec == {ServoMotor, [pwm_pin: 12, frequency: 50]}
    end
  end

  describe "multiple actuators on joint" do
    defmodule MultipleActuatorsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type :revolute

            actuator(:main_motor, MainMotor)
            actuator(:brake, {BrakeActuator, pin: 5})

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :arm do
            end
          end
        end
      end
    end

    test "multiple actuators on a single joint" do
      [link] = Info.topology(MultipleActuatorsRobot)
      [joint] = link.joints
      assert length(joint.actuators) == 2
      names = Enum.map(joint.actuators, & &1.name)
      assert :main_motor in names
      assert :brake in names
    end
  end

  describe "actuators in nested joints" do
    defmodule NestedJointActuatorsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          joint :shoulder do
            type :revolute

            actuator(:shoulder_motor, ShoulderMotor)

            limit do
              effort(~u(10 newton_meter))
              velocity(~u(100 degree_per_second))
            end

            link :upper_arm do
              joint :elbow do
                type :revolute

                actuator(:elbow_motor, ElbowMotor)

                limit do
                  effort(~u(5 newton_meter))
                  velocity(~u(90 degree_per_second))
                end

                link :forearm do
                end
              end
            end
          end
        end
      end
    end

    test "actuators in nested joints" do
      [base_link] = Info.topology(NestedJointActuatorsRobot)
      [shoulder_joint] = base_link.joints
      [shoulder_actuator] = shoulder_joint.actuators
      assert shoulder_actuator.name == :shoulder_motor

      upper_arm = shoulder_joint.link
      [elbow_joint] = upper_arm.joints
      [elbow_actuator] = elbow_joint.actuators
      assert elbow_actuator.name == :elbow_motor
    end
  end
end
