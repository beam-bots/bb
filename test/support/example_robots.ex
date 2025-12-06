# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.ExampleRobots do
  @moduledoc """
  Example robot topologies for testing and documentation.
  """

  defmodule DifferentialDriveRobot do
    @moduledoc """
    A simple two-wheeled differential drive robot with a caster.

    Structure:
    - base_link (main chassis)
      - left_wheel (continuous joint)
      - right_wheel (continuous joint)
      - caster_wheel (fixed)
    """
    use BB
    import BB.Unit

    settings do
      name(:differential_drive)
    end

    topology do
      link :base_link do
        inertial do
          mass(~u(5 kilogram))

          inertia do
            ixx(~u(0.05 kilogram_square_meter))
            iyy(~u(0.05 kilogram_square_meter))
            izz(~u(0.08 kilogram_square_meter))
            ixy(~u(0 kilogram_square_meter))
            ixz(~u(0 kilogram_square_meter))
            iyz(~u(0 kilogram_square_meter))
          end
        end

        visual do
          box do
            x(~u(0.3 meter))
            y(~u(0.2 meter))
            z(~u(0.1 meter))
          end

          material do
            name(:chassis_grey)

            color do
              red(0.5)
              green(0.5)
              blue(0.5)
              alpha(1.0)
            end
          end
        end

        collision do
          box do
            x(~u(0.3 meter))
            y(~u(0.2 meter))
            z(~u(0.1 meter))
          end
        end

        joint :left_wheel_joint do
          type(:continuous)

          origin do
            x(~u(0 meter))
            y(~u(0.12 meter))
            z(~u(-0.03 meter))
          end

          axis do
            roll(~u(-90 degree))
          end

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(360 degree_per_second))
          end

          dynamics do
            damping(~u(0.01 newton_meter_second_per_radian))
            friction(~u(0.1 newton_meter))
          end

          link :left_wheel do
            inertial do
              mass(~u(0.5 kilogram))

              inertia do
                ixx(~u(0.001 kilogram_square_meter))
                iyy(~u(0.001 kilogram_square_meter))
                izz(~u(0.001 kilogram_square_meter))
                ixy(~u(0 kilogram_square_meter))
                ixz(~u(0 kilogram_square_meter))
                iyz(~u(0 kilogram_square_meter))
              end
            end

            visual do
              origin do
                roll(~u(90 degree))
              end

              cylinder do
                radius(~u(0.05 meter))
                height(~u(0.02 meter))
              end

              material do
                name(:left_wheel_black)

                color do
                  red(0.1)
                  green(0.1)
                  blue(0.1)
                  alpha(1.0)
                end
              end
            end

            collision do
              origin do
                roll(~u(90 degree))
              end

              cylinder do
                radius(~u(0.05 meter))
                height(~u(0.02 meter))
              end
            end
          end
        end

        joint :right_wheel_joint do
          type(:continuous)

          origin do
            x(~u(0 meter))
            y(~u(-0.12 meter))
            z(~u(-0.03 meter))
          end

          axis do
            roll(~u(-90 degree))
          end

          limit do
            effort(~u(10 newton_meter))
            velocity(~u(360 degree_per_second))
          end

          dynamics do
            damping(~u(0.01 newton_meter_second_per_radian))
            friction(~u(0.1 newton_meter))
          end

          link :right_wheel do
            inertial do
              mass(~u(0.5 kilogram))

              inertia do
                ixx(~u(0.001 kilogram_square_meter))
                iyy(~u(0.001 kilogram_square_meter))
                izz(~u(0.001 kilogram_square_meter))
                ixy(~u(0 kilogram_square_meter))
                ixz(~u(0 kilogram_square_meter))
                iyz(~u(0 kilogram_square_meter))
              end
            end

            visual do
              origin do
                roll(~u(90 degree))
              end

              cylinder do
                radius(~u(0.05 meter))
                height(~u(0.02 meter))
              end

              material do
                name(:right_wheel_black)

                color do
                  red(0.1)
                  green(0.1)
                  blue(0.1)
                  alpha(1.0)
                end
              end
            end

            collision do
              origin do
                roll(~u(90 degree))
              end

              cylinder do
                radius(~u(0.05 meter))
                height(~u(0.02 meter))
              end
            end
          end
        end

        joint :caster_joint do
          type(:fixed)

          origin do
            x(~u(-0.1 meter))
            z(~u(-0.04 meter))
          end

          link :caster_wheel do
            visual do
              sphere do
                radius(~u(0.02 meter))
              end

              material do
                name(:caster_grey)

                color do
                  red(0.3)
                  green(0.3)
                  blue(0.3)
                  alpha(1.0)
                end
              end
            end

            collision do
              sphere do
                radius(~u(0.02 meter))
              end
            end
          end
        end
      end
    end
  end

  defmodule SixDofArm do
    @moduledoc """
    A 6 degree-of-freedom industrial robot arm.

    Structure:
    - base_link
      - shoulder_pan (revolute, Z-axis)
        - shoulder_lift (revolute, Y-axis)
          - elbow (revolute, Y-axis)
            - wrist_1 (revolute, Y-axis)
              - wrist_2 (revolute, Z-axis)
                - wrist_3 (revolute, Y-axis)
                  - tool0 (fixed, tool mounting point)
    """
    use BB
    import BB.Unit

    settings do
      name(:six_dof_arm)
    end

    topology do
      link :base_link do
        inertial do
          mass(~u(4 kilogram))

          inertia do
            ixx(~u(0.02 kilogram_square_meter))
            iyy(~u(0.02 kilogram_square_meter))
            izz(~u(0.02 kilogram_square_meter))
            ixy(~u(0 kilogram_square_meter))
            ixz(~u(0 kilogram_square_meter))
            iyz(~u(0 kilogram_square_meter))
          end
        end

        visual do
          cylinder do
            radius(~u(0.075 meter))
            height(~u(0.05 meter))
          end

          material do
            name(:arm_blue)

            color do
              red(0.2)
              green(0.4)
              blue(0.8)
              alpha(1.0)
            end
          end
        end

        collision do
          cylinder do
            radius(~u(0.075 meter))
            height(~u(0.05 meter))
          end
        end

        joint :shoulder_pan_joint do
          type(:revolute)

          origin do
            z(~u(0.089 meter))
          end

          axis do
          end

          limit do
            lower(~u(-180 degree))
            upper(~u(180 degree))
            effort(~u(150 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          link :shoulder_link do
            inertial do
              mass(~u(3.7 kilogram))

              origin do
                z(~u(0.05 meter))
              end

              inertia do
                ixx(~u(0.01 kilogram_square_meter))
                iyy(~u(0.01 kilogram_square_meter))
                izz(~u(0.01 kilogram_square_meter))
                ixy(~u(0 kilogram_square_meter))
                ixz(~u(0 kilogram_square_meter))
                iyz(~u(0 kilogram_square_meter))
              end
            end

            visual do
              origin do
                z(~u(0.05 meter))
              end

              cylinder do
                radius(~u(0.06 meter))
                height(~u(0.1 meter))
              end

              material do
                name(:shoulder_blue)

                color do
                  red(0.2)
                  green(0.4)
                  blue(0.8)
                  alpha(1.0)
                end
              end
            end

            joint :shoulder_lift_joint do
              type(:revolute)

              origin do
                y(~u(0.135 meter))
                z(~u(0.089 meter))
              end

              axis do
                roll(~u(-90 degree))
              end

              limit do
                lower(~u(-180 degree))
                upper(~u(180 degree))
                effort(~u(150 newton_meter))
                velocity(~u(180 degree_per_second))
              end

              link :upper_arm_link do
                inertial do
                  mass(~u(8.4 kilogram))

                  origin do
                    z(~u(0.2125 meter))
                  end

                  inertia do
                    ixx(~u(0.13 kilogram_square_meter))
                    iyy(~u(0.13 kilogram_square_meter))
                    izz(~u(0.02 kilogram_square_meter))
                    ixy(~u(0 kilogram_square_meter))
                    ixz(~u(0 kilogram_square_meter))
                    iyz(~u(0 kilogram_square_meter))
                  end
                end

                visual do
                  origin do
                    z(~u(0.2125 meter))
                  end

                  box do
                    x(~u(0.08 meter))
                    y(~u(0.08 meter))
                    z(~u(0.425 meter))
                  end

                  material do
                    name(:upper_arm_blue)

                    color do
                      red(0.2)
                      green(0.4)
                      blue(0.8)
                      alpha(1.0)
                    end
                  end
                end

                joint :elbow_joint do
                  type(:revolute)

                  origin do
                    z(~u(0.425 meter))
                  end

                  axis do
                    roll(~u(-90 degree))
                  end

                  limit do
                    lower(~u(-180 degree))
                    upper(~u(180 degree))
                    effort(~u(28 newton_meter))
                    velocity(~u(180 degree_per_second))
                  end

                  link :forearm_link do
                    inertial do
                      mass(~u(2.3 kilogram))

                      origin do
                        z(~u(0.196 meter))
                      end

                      inertia do
                        ixx(~u(0.03 kilogram_square_meter))
                        iyy(~u(0.03 kilogram_square_meter))
                        izz(~u(0.004 kilogram_square_meter))
                        ixy(~u(0 kilogram_square_meter))
                        ixz(~u(0 kilogram_square_meter))
                        iyz(~u(0 kilogram_square_meter))
                      end
                    end

                    visual do
                      origin do
                        z(~u(0.196 meter))
                      end

                      box do
                        x(~u(0.06 meter))
                        y(~u(0.06 meter))
                        z(~u(0.392 meter))
                      end

                      material do
                        name(:forearm_blue)

                        color do
                          red(0.2)
                          green(0.4)
                          blue(0.8)
                          alpha(1.0)
                        end
                      end
                    end

                    joint :wrist_1_joint do
                      type(:revolute)

                      origin do
                        y(~u(0.093 meter))
                        z(~u(0.392 meter))
                      end

                      axis do
                        roll(~u(-90 degree))
                      end

                      limit do
                        lower(~u(-180 degree))
                        upper(~u(180 degree))
                        effort(~u(12 newton_meter))
                        velocity(~u(180 degree_per_second))
                      end

                      link :wrist_1_link do
                        inertial do
                          mass(~u(1.2 kilogram))

                          inertia do
                            ixx(~u(0.002 kilogram_square_meter))
                            iyy(~u(0.002 kilogram_square_meter))
                            izz(~u(0.002 kilogram_square_meter))
                            ixy(~u(0 kilogram_square_meter))
                            ixz(~u(0 kilogram_square_meter))
                            iyz(~u(0 kilogram_square_meter))
                          end
                        end

                        visual do
                          cylinder do
                            radius(~u(0.04 meter))
                            height(~u(0.08 meter))
                          end

                          material do
                            name(:wrist_grey)

                            color do
                              red(0.6)
                              green(0.6)
                              blue(0.6)
                              alpha(1.0)
                            end
                          end
                        end

                        joint :wrist_2_joint do
                          type(:revolute)

                          origin do
                            z(~u(0.093 meter))
                          end

                          axis do
                          end

                          limit do
                            lower(~u(-180 degree))
                            upper(~u(180 degree))
                            effort(~u(12 newton_meter))
                            velocity(~u(180 degree_per_second))
                          end

                          link :wrist_2_link do
                            inertial do
                              mass(~u(1.2 kilogram))

                              inertia do
                                ixx(~u(0.002 kilogram_square_meter))
                                iyy(~u(0.002 kilogram_square_meter))
                                izz(~u(0.002 kilogram_square_meter))
                                ixy(~u(0 kilogram_square_meter))
                                ixz(~u(0 kilogram_square_meter))
                                iyz(~u(0 kilogram_square_meter))
                              end
                            end

                            visual do
                              cylinder do
                                radius(~u(0.04 meter))
                                height(~u(0.08 meter))
                              end

                              material do
                                name(:wrist_2_grey)

                                color do
                                  red(0.6)
                                  green(0.6)
                                  blue(0.6)
                                  alpha(1.0)
                                end
                              end
                            end

                            joint :wrist_3_joint do
                              type(:revolute)

                              origin do
                                y(~u(0.093 meter))
                              end

                              axis do
                                roll(~u(-90 degree))
                              end

                              limit do
                                lower(~u(-180 degree))
                                upper(~u(180 degree))
                                effort(~u(12 newton_meter))
                                velocity(~u(180 degree_per_second))
                              end

                              link :wrist_3_link do
                                inertial do
                                  mass(~u(0.2 kilogram))

                                  inertia do
                                    ixx(~u(0.0001 kilogram_square_meter))
                                    iyy(~u(0.0001 kilogram_square_meter))
                                    izz(~u(0.0001 kilogram_square_meter))
                                    ixy(~u(0 kilogram_square_meter))
                                    ixz(~u(0 kilogram_square_meter))
                                    iyz(~u(0 kilogram_square_meter))
                                  end
                                end

                                visual do
                                  cylinder do
                                    radius(~u(0.03 meter))
                                    height(~u(0.04 meter))
                                  end

                                  material do
                                    name(:wrist_3_grey)

                                    color do
                                      red(0.6)
                                      green(0.6)
                                      blue(0.6)
                                      alpha(1.0)
                                    end
                                  end
                                end

                                joint :tool0_joint do
                                  type(:fixed)

                                  origin do
                                    z(~u(0.082 meter))
                                  end

                                  link(:tool0)
                                end
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end
  end

  defmodule PanTiltCamera do
    @moduledoc """
    A simple pan-tilt camera mount.

    Structure:
    - base_link
      - pan_joint (revolute, Z-axis)
        - pan_link
          - tilt_joint (revolute, Y-axis)
            - camera_link
    """
    use BB
    import BB.Unit

    settings do
      name(:pan_tilt_camera)
    end

    topology do
      link :base_link do
        inertial do
          mass(~u(0.5 kilogram))

          inertia do
            ixx(~u(0.001 kilogram_square_meter))
            iyy(~u(0.001 kilogram_square_meter))
            izz(~u(0.001 kilogram_square_meter))
            ixy(~u(0 kilogram_square_meter))
            ixz(~u(0 kilogram_square_meter))
            iyz(~u(0 kilogram_square_meter))
          end
        end

        visual do
          cylinder do
            radius(~u(0.03 meter))
            height(~u(0.02 meter))
          end

          material do
            name(:base_black)

            color do
              red(0.1)
              green(0.1)
              blue(0.1)
              alpha(1.0)
            end
          end
        end

        joint :pan_joint do
          type(:revolute)

          origin do
            z(~u(0.015 meter))
          end

          axis do
          end

          limit do
            lower(~u(-170 degree))
            upper(~u(170 degree))
            effort(~u(2 newton_meter))
            velocity(~u(90 degree_per_second))
          end

          link :pan_link do
            inertial do
              mass(~u(0.1 kilogram))

              inertia do
                ixx(~u(0.0001 kilogram_square_meter))
                iyy(~u(0.0001 kilogram_square_meter))
                izz(~u(0.0001 kilogram_square_meter))
                ixy(~u(0 kilogram_square_meter))
                ixz(~u(0 kilogram_square_meter))
                iyz(~u(0 kilogram_square_meter))
              end
            end

            visual do
              origin do
                z(~u(0.015 meter))
              end

              box do
                x(~u(0.04 meter))
                y(~u(0.04 meter))
                z(~u(0.03 meter))
              end

              material do
                name(:pan_black)

                color do
                  red(0.1)
                  green(0.1)
                  blue(0.1)
                  alpha(1.0)
                end
              end
            end

            joint :tilt_joint do
              type(:revolute)

              origin do
                z(~u(0.035 meter))
              end

              axis do
                roll(~u(-90 degree))
              end

              limit do
                lower(~u(-45 degree))
                upper(~u(90 degree))
                effort(~u(1 newton_meter))
                velocity(~u(60 degree_per_second))
              end

              link :camera_link do
                inertial do
                  mass(~u(0.15 kilogram))

                  origin do
                    x(~u(0.02 meter))
                  end

                  inertia do
                    ixx(~u(0.0001 kilogram_square_meter))
                    iyy(~u(0.0001 kilogram_square_meter))
                    izz(~u(0.0001 kilogram_square_meter))
                    ixy(~u(0 kilogram_square_meter))
                    ixz(~u(0 kilogram_square_meter))
                    iyz(~u(0 kilogram_square_meter))
                  end
                end

                visual do
                  origin do
                    x(~u(0.02 meter))
                  end

                  box do
                    x(~u(0.04 meter))
                    y(~u(0.06 meter))
                    z(~u(0.03 meter))
                  end

                  material do
                    name(:camera_silver)

                    color do
                      red(0.7)
                      green(0.7)
                      blue(0.7)
                      alpha(1.0)
                    end
                  end
                end

                collision do
                  origin do
                    x(~u(0.02 meter))
                  end

                  box do
                    x(~u(0.04 meter))
                    y(~u(0.06 meter))
                    z(~u(0.03 meter))
                  end
                end
              end
            end
          end
        end
      end
    end
  end

  defmodule LinearActuator do
    @moduledoc """
    A simple linear actuator (prismatic joint example).

    Structure:
    - base_link
      - slider_joint (prismatic, Z-axis)
        - slider_link
    """
    use BB
    import BB.Unit

    settings do
      name(:linear_actuator)
    end

    topology do
      link :base_link do
        inertial do
          mass(~u(2 kilogram))

          inertia do
            ixx(~u(0.01 kilogram_square_meter))
            iyy(~u(0.01 kilogram_square_meter))
            izz(~u(0.005 kilogram_square_meter))
            ixy(~u(0 kilogram_square_meter))
            ixz(~u(0 kilogram_square_meter))
            iyz(~u(0 kilogram_square_meter))
          end
        end

        visual do
          box do
            x(~u(0.1 meter))
            y(~u(0.1 meter))
            z(~u(0.3 meter))
          end

          material do
            name(:actuator_blue)

            color do
              red(0.2)
              green(0.3)
              blue(0.7)
              alpha(1.0)
            end
          end
        end

        collision do
          box do
            x(~u(0.1 meter))
            y(~u(0.1 meter))
            z(~u(0.3 meter))
          end
        end

        joint :slider_joint do
          type(:prismatic)

          origin do
            z(~u(0.2 meter))
          end

          axis do
          end

          limit do
            lower(~u(0 meter))
            upper(~u(0.2 meter))
            effort(~u(100 newton))
            velocity(~u(0.1 meter_per_second))
          end

          dynamics do
            damping(~u(10 newton_second_per_meter))
            friction(~u(5 newton))
          end

          link :slider_link do
            inertial do
              mass(~u(0.5 kilogram))

              inertia do
                ixx(~u(0.001 kilogram_square_meter))
                iyy(~u(0.001 kilogram_square_meter))
                izz(~u(0.001 kilogram_square_meter))
                ixy(~u(0 kilogram_square_meter))
                ixz(~u(0 kilogram_square_meter))
                iyz(~u(0 kilogram_square_meter))
              end
            end

            visual do
              cylinder do
                radius(~u(0.03 meter))
                height(~u(0.15 meter))
              end

              material do
                name(:slider_silver)

                color do
                  red(0.8)
                  green(0.8)
                  blue(0.8)
                  alpha(1.0)
                end
              end
            end

            collision do
              cylinder do
                radius(~u(0.03 meter))
                height(~u(0.15 meter))
              end
            end
          end
        end
      end
    end
  end
end
