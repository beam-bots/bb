# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix.Dsl do
  @moduledoc """
  The DSL extension for describing robot properties and topologies.
  """
  alias Spark.Dsl.Entity
  alias Spark.Dsl.Section
  import Kinetix.Unit
  import Kinetix.Unit.Option

  @origin %Entity{
    name: :origin,
    target: Kinetix.Dsl.Origin,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    schema: [
      roll: [
        type: unit_type(compatible: :degree),
        doc: "rotation around the `x` axis",
        required: false,
        default: ~u(0 degree)
      ],
      pitch: [
        type: unit_type(compatible: :degree),
        doc: "rotation around the `y` axis",
        required: false,
        default: ~u(0 degree)
      ],
      yaw: [
        type: unit_type(compatible: :degree),
        doc: "rotation around the `z` axis",
        required: false,
        default: ~u(0 degree)
      ],
      x: [
        type: unit_type(compatible: :meter),
        doc: "translation along the `x` axis",
        required: false,
        default: ~u(0 meter)
      ],
      y: [
        type: unit_type(compatible: :meter),
        doc: "translation along the `y` axis",
        required: false,
        default: ~u(0 meter)
      ],
      z: [
        type: unit_type(compatible: :meter),
        doc: "translation along the `z` axis",
        required: false,
        default: ~u(0 meter)
      ]
    ]
  }

  @axis %Entity{
    name: :axis,
    target: Kinetix.Dsl.Axis,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    schema: [
      x: [
        type: unit_type(compatible: :meter),
        doc: "translation along the `x` axis",
        required: false,
        default: ~u(0 meter)
      ],
      y: [
        type: unit_type(compatible: :meter),
        doc: "translation along the `y` axis",
        required: false,
        default: ~u(0 meter)
      ],
      z: [
        type: unit_type(compatible: :meter),
        doc: "translation along the `z` axis",
        required: false,
        default: ~u(0 meter)
      ]
    ]
  }

  @dynamics %Entity{
    name: :dynamics,
    describe: """
    An element specifying physical properties of the joint. These values are used to specify modeling properties of the joint, particularly useful for simulation.
    """,
    target: Kinetix.Dsl.Dynamics,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    schema: [
      damping: [
        type:
          {:or,
           [
             unit_type(compatible: :newton_second_per_meter),
             unit_type(compatible: :newton_meter_second_per_degree)
           ]},
        doc: "The physical damping value of the joint",
        required: false
      ],
      friction: [
        type: {:or, [unit_type(compatible: :newton), unit_type(compatible: :newton_meter)]},
        doc: "The physical static friction value of the joint",
        required: false
      ]
    ]
  }

  @limit %Entity{
    name: :limit,
    describe: "Limits applied to joint movement",
    target: Kinetix.Dsl.Limit,
    imports: [Kinetix.Unit],
    schema: [
      lower: [
        type: {:or, [unit_type(compatible: :degree), unit_type(compatible: :meter)]},
        doc: "The lower joint limit",
        required: false
      ],
      upper: [
        type: {:or, [unit_type(compatible: :degree), unit_type(compatible: :meter)]},
        doc: "The upper joint limit",
        required: false
      ],
      effort: [
        type: unit_type(compatible: :newton_meter),
        doc:
          "The maximum effort - both positive and negative - that can be commanded to the joint",
        required: true
      ],
      velocity: [
        type:
          {:or,
           [
             unit_type(compatible: :degree_per_second),
             unit_type(compatible: :meter_per_second)
           ]},
        doc: "Maximum velocity - both positive and negative - that can be commanded to the joint",
        required: true
      ]
    ]
  }

  @sensor %Entity{
    name: :sensor,
    describe: "A sensor attached to the robot, a link, or a joint.",
    target: Kinetix.Dsl.Sensor,
    identifier: :name,
    args: [:name, :child_spec],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the sensor"
      ],
      child_spec: [
        type: {:or, [:module, {:tuple, [:module, :keyword_list]}]},
        required: true,
        doc:
          "The child specification for the sensor process. Either a module or `{module, keyword_list}`"
      ]
    ]
  }

  @actuator %Entity{
    name: :actuator,
    describe: "An actuator attached to a joint.",
    target: Kinetix.Dsl.Actuator,
    identifier: :name,
    args: [:name, :child_spec],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the actuator"
      ],
      child_spec: [
        type: {:or, [:module, {:tuple, [:module, :keyword_list]}]},
        required: true,
        doc:
          "The child specification for the actuator process. Either a module or `{module, keyword_list}`"
      ]
    ]
  }

  @joint %Entity{
    name: :joint,
    describe: """
    A kinematic joint between a parent link and a child link.
    """,
    target: Kinetix.Dsl.Joint,
    identifier: :name,
    imports: [Kinetix.Unit],
    args: [:name],
    entities: [
      link: [],
      origin: [
        %{
          @origin
          | describe: """
            This is the transform from the parent link to the child link. The joint is located at the origin of the child link, as shown in the figure above
            """
        }
      ],
      axis: [
        %{
          @axis
          | describe: """
            The joint axis specified in the joint frame. This is the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints. The axis is specified in the joint frame of reference. Fixed and floating joints do not use the axis field
            """
        }
      ],
      dynamics: [@dynamics],
      limit: [@limit],
      sensors: [@sensor],
      actuators: [@actuator]
    ],
    recursive_as: :joints,
    singleton_entity_keys: [:dynamics, :origin, :axis, :link, :limit],
    schema: [
      name: [
        type: :atom,
        required: false,
        doc: "A unique name for the joint"
      ],
      type: [
        type: {:in, [:revolute, :continuous, :prismatic, :fixed, :floating, :planar]},
        doc: "Specifies the type of joint"
      ]
    ]
  }

  @box %Entity{
    name: :box,
    describe: "Box geometry",
    target: Kinetix.Dsl.Box,
    imports: [Kinetix.Unit],
    schema: [
      x: [
        type: unit_type(compatible: :meter),
        doc: "The length of the X axis side",
        required: true
      ],
      y: [
        type: unit_type(compatible: :meter),
        doc: "The length of the Y axis side",
        required: true
      ],
      z: [
        type: unit_type(compatible: :meter),
        doc: "The length of the Z axis side",
        required: true
      ]
    ]
  }

  @cylinder %Entity{
    name: :cylinder,
    describe: """
    A cylindrical geometry

    The origin of the cylinder is the center.
    """,
    target: Kinetix.Dsl.Cylinder,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    schema: [
      radius: [
        type: unit_type(compatible: :meter),
        doc: "The distance from the center to the circumference",
        required: true
      ],
      height: [
        type: unit_type(compatible: :meter),
        doc: "The height of the cylinder",
        required: true
      ]
    ]
  }

  @sphere %Entity{
    name: :sphere,
    describe: """
    A spherical geometry

    The origin of the sphere is its center.
    """,
    target: Kinetix.Dsl.Sphere,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    schema: [
      radius: [
        type: unit_type(compatible: :meter),
        doc: "The distance from the center of the sphere to your edge",
        required: true
      ]
    ]
  }

  @mesh %Entity{
    name: :mesh,
    describe: """
    A mesh object specified by a filename
    """,
    target: Kinetix.Dsl.Mesh,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    schema: [
      filename: [
        type: :string,
        doc: "The path to the 3D model",
        required: true
      ],
      scale: [
        type: :number,
        doc: "A scale factor for the mest",
        required: false,
        default: 1
      ]
    ]
  }

  @color %Entity{
    name: :color,
    describe: """
    The color of the meterial
    """,
    target: Kinetix.Dsl.Color,
    identifier: {:auto, :unique_integer},
    schema: [
      red: [
        type: {:custom, Kinetix.Dsl.Color, :validate, []},
        doc: "The red element of the color",
        required: true
      ],
      green: [
        type: {:custom, Kinetix.Dsl.Color, :validate, []},
        doc: "The green element of the color",
        required: true
      ],
      blue: [
        type: {:custom, Kinetix.Dsl.Color, :validate, []},
        doc: "The blue element of the color",
        required: true
      ],
      alpha: [
        type: {:custom, Kinetix.Dsl.Color, :validate, []},
        doc: "The alpha element of the color",
        required: true
      ]
    ]
  }

  @texture %Entity{
    name: :texture,
    describe: """
    A texture to apply to the material
    """,
    target: Kinetix.Dsl.Texture,
    identifier: {:auto, :unique_integer},
    schema: [
      filename: [
        type: :string,
        doc: "The image file to use",
        required: true
      ]
    ]
  }

  @material %Entity{
    name: :material,
    describe: """
    The material of the visual element
    """,
    target: Kinetix.Dsl.Material,
    identifier: {:auto, :unique_integer},
    entities: [color: [@color], texture: [@texture]],
    singleton_entity_keys: [:color, :texture],
    schema: [
      name: [
        type: :atom,
        doc: "The name of the material",
        required: true
      ]
    ]
  }

  @visual %Entity{
    name: :visual,
    describe: """
    Visual attributes for a link.
    """,
    target: Kinetix.Dsl.Visual,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    entities: [
      geometry: [@box, @cylinder, @sphere, @mesh],
      material: [@material],
      origin: [
        %{
          @origin
          | describe:
              "The refrence frame of the visual element with respect to the reference frame of the link"
        }
      ]
    ],
    singleton_entity_keys: [:geometry, :material, :origin]
  }

  @inertia %Entity{
    name: :inertia,
    describe: """
    How the link resists rotational motion.
    """,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    target: Kinetix.Dsl.Inertia,
    schema: [
      ixx: [
        type: unit_type(compatible: :kilogram_square_meter),
        doc: "Resistance to rotation around the x-axis",
        required: true
      ],
      iyy: [
        type: unit_type(compatible: :kilogram_square_meter),
        doc: "Resistance to rotation around the y-axis",
        required: true
      ],
      izz: [
        type: unit_type(compatible: :kilogram_square_meter),
        doc: "Resistance to rotation around the z-axis",
        required: true
      ],
      ixy: [
        type: unit_type(compatible: :kilogram_square_meter),
        doc: "Coupling between the x and y axes",
        required: true
      ],
      ixz: [
        type: unit_type(compatible: :kilogram_square_meter),
        doc: "Coupling between the x and z axes",
        required: true
      ],
      iyz: [
        type: unit_type(compatible: :kilogram_square_meter),
        doc: "Coupling between the y and z axes",
        required: true
      ]
    ]
  }

  @inertial %Entity{
    name: :inertial,
    describe: """
    A link's mass, position of it's center of mass and it's central inertia properties
    """,
    target: Kinetix.Dsl.Inertial,
    identifier: {:auto, :unique_integer},
    imports: [Kinetix.Unit],
    entities: [
      origin: [
        %{
          @origin
          | describe:
              "Specifies where the link's center of mass is located, relative to the link's reference frame"
        }
      ],
      inertia: [@inertia]
    ],
    singleton_entity_keys: [:origin, :inertia],
    schema: [
      mass: [
        type: unit_type(compatible: :kilogram),
        doc: "The mass of the link",
        required: true
      ]
    ]
  }

  @collision %Entity{
    name: :collision,
    describe: """
    The collision properties of a link.
    """,
    target: Kinetix.Dsl.Collision,
    imports: [Kinetix.Unit],
    entities: [
      origin: [
        %{
          @origin
          | describe:
              "The refrence frame of the collision element, relative to the reference frame of the link"
        }
      ],
      geometry: [@box, @cylinder, @sphere, @mesh]
    ],
    singleton_entity_keys: [:origin, :geometry],
    schema: [
      name: [
        type: :atom,
        doc: "An optional name of the link geometry",
        required: false
      ]
    ]
  }

  @link %Entity{
    name: :link,
    describe: """
    A kinematic link (ie solid body).
    """,
    target: Kinetix.Dsl.Link,
    identifier: :name,
    imports: [Kinetix.Unit],
    args: [:name],
    recursive_as: :link,
    entities: [
      joints: [],
      inertial: [@inertial],
      visual: [@visual],
      collisions: [@collision],
      sensors: [@sensor]
    ],
    singleton_entity_keys: [:visual, :inertial],
    schema: [
      name: [
        type: :atom,
        doc: "The name of the link"
      ]
    ]
  }

  @settings %Section{
    name: :settings,
    describe: "System-wide settings",
    schema: [
      registry_module: [
        type: :module,
        doc: "The registry module to use",
        required: false,
        default: Registry
      ],
      registry_options: [
        type: :keyword_list,
        doc:
          "Options passed to Registry.start_link/1. Defaults to `[partitions: System.schedulers_online()]` at runtime.",
        required: false
      ],
      supervisor_module: [
        type: :module,
        doc: "The supervisor module to use",
        required: false,
        default: Supervisor
      ]
    ]
  }

  @robot %Section{
    name: :robot,
    describe: "Describe universal robot properties",
    entities: [@link, @joint, @sensor],
    imports: [Kinetix.Unit],
    sections: [@settings],
    schema: [
      name: [
        type: :atom,
        required: false,
        doc: "The name of the robot, defaults to the name of the defining module"
      ]
    ]
  }

  use Spark.Dsl.Extension,
    sections: [@robot],
    transformers: [
      __MODULE__.DefaultNameTransformer,
      __MODULE__.LinkTransformer,
      __MODULE__.SupervisorTransformer,
      __MODULE__.RobotTransformer
    ]
end
