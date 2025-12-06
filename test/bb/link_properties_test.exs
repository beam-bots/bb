# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.LinkPropertiesTest do
  use ExUnit.Case, async: true
  alias BB.Dsl.{Collision, Inertia, Inertial, Info, Origin, Visual}
  import BB.Unit

  describe "inertial" do
    defmodule InertialRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          inertial do
            mass(~u(1.5 kilogram))

            origin do
              x ~u(0.1 meter)
              y ~u(0.2 meter)
              z ~u(0.05 meter)
            end

            inertia do
              ixx(~u(0.001 kilogram_square_meter))
              iyy(~u(0.002 kilogram_square_meter))
              izz(~u(0.003 kilogram_square_meter))
              ixy(~u(0 kilogram_square_meter))
              ixz(~u(0 kilogram_square_meter))
              iyz(~u(0 kilogram_square_meter))
            end
          end
        end
      end
    end

    test "inertial with mass compiles" do
      [link] = Info.topology(InertialRobot)
      assert is_struct(link.inertial, Inertial)
      assert link.inertial.mass == ~u(1.5 kilogram)
    end

    test "inertial with origin (centre of mass offset)" do
      [link] = Info.topology(InertialRobot)
      assert is_struct(link.inertial.origin, Origin)
      assert link.inertial.origin.x == ~u(0.1 meter)
      assert link.inertial.origin.y == ~u(0.2 meter)
      assert link.inertial.origin.z == ~u(0.05 meter)
    end

    test "inertial with full inertia tensor" do
      [link] = Info.topology(InertialRobot)
      assert is_struct(link.inertial.inertia, Inertia)
      assert link.inertial.inertia.ixx == ~u(0.001 kilogram_square_meter)
      assert link.inertial.inertia.iyy == ~u(0.002 kilogram_square_meter)
      assert link.inertial.inertia.izz == ~u(0.003 kilogram_square_meter)
      assert link.inertial.inertia.ixy == ~u(0 kilogram_square_meter)
    end

    defmodule InertialWithoutOriginRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          inertial do
            mass(~u(2 kilogram))
          end
        end
      end
    end

    test "inertial without origin compiles" do
      [link] = Info.topology(InertialWithoutOriginRobot)
      assert is_struct(link.inertial, Inertial)
      assert link.inertial.mass == ~u(2 kilogram)
      assert is_nil(link.inertial.origin)
    end
  end

  describe "visual" do
    defmodule VisualBoxRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          visual do
            origin do
              z ~u(0.05 meter)
            end

            box do
              x ~u(0.1 meter)
              y ~u(0.2 meter)
              z ~u(0.1 meter)
            end
          end
        end
      end
    end

    test "visual with box geometry" do
      [link] = Info.topology(VisualBoxRobot)
      assert is_struct(link.visual, Visual)
      assert link.visual.geometry.x == ~u(0.1 meter)
      assert link.visual.geometry.y == ~u(0.2 meter)
      assert link.visual.geometry.z == ~u(0.1 meter)
    end

    test "visual with origin offset" do
      [link] = Info.topology(VisualBoxRobot)
      assert link.visual.origin.z == ~u(0.05 meter)
    end

    defmodule VisualCylinderRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          visual do
            cylinder do
              radius(~u(0.05 meter))
              height(~u(0.3 meter))
            end
          end
        end
      end
    end

    test "visual with cylinder geometry" do
      [link] = Info.topology(VisualCylinderRobot)
      assert link.visual.geometry.radius == ~u(0.05 meter)
      assert link.visual.geometry.height == ~u(0.3 meter)
    end

    defmodule VisualSphereRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          visual do
            sphere do
              radius(~u(0.1 meter))
            end
          end
        end
      end
    end

    test "visual with sphere geometry" do
      [link] = Info.topology(VisualSphereRobot)
      assert link.visual.geometry.radius == ~u(0.1 meter)
    end

    defmodule VisualMeshRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          visual do
            mesh do
              filename("meshes/base_link.stl")
              scale(0.001)
            end
          end
        end
      end
    end

    test "visual with mesh geometry" do
      [link] = Info.topology(VisualMeshRobot)
      assert link.visual.geometry.filename == "meshes/base_link.stl"
      assert link.visual.geometry.scale == 0.001
    end

    defmodule VisualWithMaterialRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          visual do
            box do
              x ~u(0.1 meter)
              y ~u(0.1 meter)
              z ~u(0.1 meter)
            end

            material do
              name :steel

              color do
                red(0.8)
                green(0.8)
                blue(0.8)
                alpha(1.0)
              end
            end
          end
        end
      end
    end

    test "visual with material (name only)" do
      [link] = Info.topology(VisualWithMaterialRobot)
      assert link.visual.material.name == :steel
    end

    test "visual with material and colour" do
      [link] = Info.topology(VisualWithMaterialRobot)
      assert link.visual.material.color.red == 0.8
      assert link.visual.material.color.green == 0.8
      assert link.visual.material.color.blue == 0.8
      assert link.visual.material.color.alpha == 1.0
    end

    defmodule VisualWithTextureRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          visual do
            box do
              x ~u(0.1 meter)
              y ~u(0.1 meter)
              z ~u(0.1 meter)
            end

            material do
              name :textured_material

              texture do
                filename("textures/wood.png")
              end
            end
          end
        end
      end
    end

    test "visual with material and texture" do
      [link] = Info.topology(VisualWithTextureRobot)
      assert link.visual.material.texture.filename == "textures/wood.png"
    end
  end

  describe "collision" do
    defmodule SingleCollisionRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          collision do
            name :base_collision

            origin do
              z ~u(0.05 meter)
            end

            box do
              x ~u(0.12 meter)
              y ~u(0.22 meter)
              z ~u(0.12 meter)
            end
          end
        end
      end
    end

    test "single collision geometry" do
      [link] = Info.topology(SingleCollisionRobot)
      assert length(link.collisions) == 1
      [collision] = link.collisions
      assert is_struct(collision, Collision)
    end

    test "collision with explicit name" do
      [link] = Info.topology(SingleCollisionRobot)
      [collision] = link.collisions
      assert collision.name == :base_collision
    end

    test "collision with origin offset" do
      [link] = Info.topology(SingleCollisionRobot)
      [collision] = link.collisions
      assert collision.origin.z == ~u(0.05 meter)
    end

    test "collision with box geometry" do
      [link] = Info.topology(SingleCollisionRobot)
      [collision] = link.collisions
      assert collision.geometry.x == ~u(0.12 meter)
    end

    defmodule MultipleCollisionsRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          collision do
            box do
              x ~u(0.1 meter)
              y ~u(0.1 meter)
              z ~u(0.1 meter)
            end
          end

          collision do
            origin do
              x ~u(0.2 meter)
            end

            sphere do
              radius(~u(0.05 meter))
            end
          end
        end
      end
    end

    test "multiple collision geometries on one link" do
      [link] = Info.topology(MultipleCollisionsRobot)
      assert length(link.collisions) == 2
    end

    defmodule CollisionCylinderRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          collision do
            cylinder do
              radius(~u(0.1 meter))
              height(~u(0.5 meter))
            end
          end
        end
      end
    end

    test "collision with cylinder geometry" do
      [link] = Info.topology(CollisionCylinderRobot)
      [collision] = link.collisions
      assert collision.geometry.radius == ~u(0.1 meter)
      assert collision.geometry.height == ~u(0.5 meter)
    end

    defmodule CollisionMeshRobot do
      @moduledoc false
      use BB

      topology do
        link :base_link do
          collision do
            mesh do
              filename("meshes/collision.stl")
            end
          end
        end
      end
    end

    test "collision with mesh geometry" do
      [link] = Info.topology(CollisionMeshRobot)
      [collision] = link.collisions
      assert collision.geometry.filename == "meshes/collision.stl"
      assert collision.geometry.scale == 1
    end
  end
end
