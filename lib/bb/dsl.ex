# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl do
  @moduledoc """
  The DSL extension for describing robot properties and topologies.
  """
  alias Spark.Dsl.Entity
  alias Spark.Dsl.Section
  import BB.Unit
  import BB.Unit.Option

  @origin %Entity{
    name: :origin,
    target: BB.Dsl.Origin,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit, BB.Dsl.ParamRef],
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
    describe: """
    Joint axis orientation specified as Euler angles.

    The axis defines the direction of rotation (for revolute joints) or
    translation (for prismatic joints). By default, the axis points along
    the Z direction. Use roll, pitch, and yaw to rotate it to the desired
    orientation.
    """,
    target: BB.Dsl.Axis,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit, BB.Dsl.ParamRef],
    schema: [
      roll: [
        type: unit_type(compatible: :degree),
        doc: "rotation around the X axis",
        required: false,
        default: ~u(0 degree)
      ],
      pitch: [
        type: unit_type(compatible: :degree),
        doc: "rotation around the Y axis",
        required: false,
        default: ~u(0 degree)
      ],
      yaw: [
        type: unit_type(compatible: :degree),
        doc: "rotation around the Z axis",
        required: false,
        default: ~u(0 degree)
      ]
    ]
  }

  @dynamics %Entity{
    name: :dynamics,
    describe: """
    An element specifying physical properties of the joint. These values are used to specify modeling properties of the joint, particularly useful for simulation.
    """,
    target: BB.Dsl.Dynamics,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit, BB.Dsl.ParamRef],
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
    target: BB.Dsl.Limit,
    imports: [BB.Unit, BB.Dsl.ParamRef],
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
        type: {:or, [unit_type(compatible: :newton), unit_type(compatible: :newton_meter)]},
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

  @param %Entity{
    name: :param,
    describe: "A runtime-adjustable parameter.",
    target: BB.Dsl.Param,
    identifier: :name,
    args: [:name],
    imports: [BB.Unit],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the parameter"
      ],
      type: [
        type: {:custom, BB.Parameter.Type, :validate, []},
        required: true,
        doc:
          "The parameter value type (:float, :integer, :boolean, :string, :atom, or {:unit, unit_type})"
      ],
      default: [
        type: :any,
        required: false,
        doc: "Default value for the parameter"
      ],
      min: [
        type: :number,
        required: false,
        doc: "Minimum value for numeric parameters"
      ],
      max: [
        type: :number,
        required: false,
        doc: "Maximum value for numeric parameters"
      ],
      doc: [
        type: :string,
        required: false,
        doc: "Documentation for the parameter"
      ]
    ]
  }

  @sensor %Entity{
    name: :sensor,
    describe: "A sensor attached to the robot, a link, or a joint.",
    target: BB.Dsl.Sensor,
    identifier: :name,
    args: [:name, :child_spec],
    imports: [BB.Dsl.ParamRef],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the sensor"
      ],
      child_spec: [
        type:
          {:or, [{:behaviour, BB.Sensor}, {:tuple, [{:behaviour, BB.Sensor}, :keyword_list]}]},
        required: true,
        doc:
          "The child specification for the sensor process. Either a module or `{module, keyword_list}`"
      ]
    ]
  }

  @actuator %Entity{
    name: :actuator,
    describe: "An actuator attached to a joint.",
    target: BB.Dsl.Actuator,
    identifier: :name,
    args: [:name, :child_spec],
    imports: [BB.Dsl.ParamRef],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the actuator"
      ],
      child_spec: [
        type:
          {:or, [{:behaviour, BB.Actuator}, {:tuple, [{:behaviour, BB.Actuator}, :keyword_list]}]},
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
    target: BB.Dsl.Joint,
    identifier: :name,
    imports: [BB.Unit, BB.Dsl.ParamRef],
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
    target: BB.Dsl.Box,
    imports: [BB.Unit],
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
    target: BB.Dsl.Cylinder,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit],
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
    target: BB.Dsl.Sphere,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit],
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
    target: BB.Dsl.Mesh,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit],
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
    target: BB.Dsl.Color,
    identifier: {:auto, :unique_integer},
    schema: [
      red: [
        type: {:custom, BB.Dsl.Color, :validate, []},
        doc: "The red element of the color",
        required: true
      ],
      green: [
        type: {:custom, BB.Dsl.Color, :validate, []},
        doc: "The green element of the color",
        required: true
      ],
      blue: [
        type: {:custom, BB.Dsl.Color, :validate, []},
        doc: "The blue element of the color",
        required: true
      ],
      alpha: [
        type: {:custom, BB.Dsl.Color, :validate, []},
        doc: "The alpha element of the color",
        required: false,
        default: 1
      ]
    ]
  }

  @texture %Entity{
    name: :texture,
    describe: """
    A texture to apply to the material
    """,
    target: BB.Dsl.Texture,
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
    target: BB.Dsl.Material,
    identifier: {:auto, :unique_integer},
    entities: [color: [@color], texture: [@texture]],
    singleton_entity_keys: [:color, :texture],
    schema: [
      name: [
        type: :atom,
        doc: "The name of the material",
        required: false
      ]
    ]
  }

  @visual %Entity{
    name: :visual,
    describe: """
    Visual attributes for a link.
    """,
    target: BB.Dsl.Visual,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit],
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
    imports: [BB.Unit, BB.Dsl.ParamRef],
    target: BB.Dsl.Inertia,
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
    target: BB.Dsl.Inertial,
    identifier: {:auto, :unique_integer},
    imports: [BB.Unit, BB.Dsl.ParamRef],
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
    target: BB.Dsl.Collision,
    imports: [BB.Unit],
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
    target: BB.Dsl.Link,
    identifier: :name,
    imports: [BB.Unit, BB.Dsl.ParamRef],
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
      name: [
        type: :atom,
        required: false,
        doc: "The name of the robot, defaults to the name of the defining module"
      ],
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
      ],
      parameter_store: [
        type:
          {:or,
           [
             {:behaviour, BB.Parameter.Store},
             {:tuple, [{:behaviour, BB.Parameter.Store}, :keyword_list]}
           ]},
        doc: "Optional parameter persistence backend. Use a module or `{Module, opts}` tuple.",
        required: false
      ],
      auto_disarm_on_error: [
        type: :boolean,
        doc:
          "Automatically disarm the robot when a hardware error is reported. Defaults to true.",
        required: false,
        default: true
      ]
    ]
  }

  @sensors %Section{
    name: :sensors,
    describe: "Robot-level sensors",
    entities: [@sensor]
  }

  @controller %Entity{
    name: :controller,
    describe: "A controller process at the robot level.",
    target: BB.Dsl.Controller,
    identifier: :name,
    args: [:name, :child_spec],
    imports: [BB.Dsl.ParamRef],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the controller"
      ],
      child_spec: [
        type:
          {:or,
           [{:behaviour, BB.Controller}, {:tuple, [{:behaviour, BB.Controller}, :keyword_list]}]},
        required: true,
        doc:
          "The child specification for the controller process. Either a module or `{module, keyword_list}`"
      ],
      simulation: [
        type: {:in, [:omit, :mock, :start]},
        default: :omit,
        doc:
          "Behaviour in simulation mode: :omit (don't start), :mock (start no-op mock), :start (start real controller)"
      ]
    ]
  }

  @controllers %Section{
    name: :controllers,
    describe: "Robot-level controllers",
    entities: [@controller],
    imports: [BB.Dsl.ParamRef]
  }

  @command_argument %Entity{
    name: :argument,
    describe: "An argument for the command.",
    target: BB.Dsl.Command.Argument,
    identifier: :name,
    args: [:name, :type],
    docs: """
    Command arguments support flexible type specifications:

    - Simple types: `:float`, `:integer`, `:boolean`, `:atom`, `:string`
    - Enums: `{:in, [:value1, :value2]}`
    - Maps: `{:map, [x: :float, y: :float, z: :float]}`
    - Modules: `MyModule`
    """,
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the argument"
      ],
      type: [
        type: :any,
        required: true,
        doc: "The type of the argument"
      ],
      required: [
        type: :boolean,
        required: false,
        default: false,
        doc: "Whether this argument is required"
      ],
      default: [
        type: :any,
        required: false,
        doc: "Default value if not provided"
      ],
      doc: [
        type: :string,
        required: false,
        doc: "Documentation for the argument"
      ]
    ]
  }

  @command %Entity{
    name: :command,
    describe: """
    A command that can be executed on the robot.

    Commands follow the Goal → Feedback → Result pattern and integrate with
    the robot's state machine to control when they can run.
    """,
    target: BB.Dsl.Command,
    identifier: :name,
    args: [:name],
    entities: [arguments: [@command_argument]],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the command"
      ],
      handler: [
        type: :module,
        required: true,
        doc: "The handler module implementing the `BB.Command` behaviour"
      ],
      timeout: [
        type: {:or, [:pos_integer, {:in, [:infinity]}]},
        required: false,
        default: :infinity,
        doc: "Timeout for command execution in milliseconds"
      ],
      allowed_states: [
        type: {:list, :atom},
        required: false,
        default: [:idle],
        doc:
          "Robot states in which this command can run. If `:executing` is included, the command can preempt running commands."
      ]
    ]
  }

  @commands %Section{
    name: :commands,
    describe: "Robot commands with Goal → Feedback → Result semantics",
    entities: [@command]
  }

  @param_group %Entity{
    name: :group,
    describe: "A group of runtime-adjustable parameters.",
    target: BB.Dsl.ParamGroup,
    identifier: :name,
    args: [:name],
    recursive_as: :groups,
    entities: [
      params: [@param],
      groups: []
    ],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the parameter group"
      ],
      doc: [
        type: :string,
        required: false,
        doc: "Documentation for the parameter group"
      ]
    ]
  }

  @bridge %Entity{
    name: :bridge,
    describe: """
    A parameter protocol bridge for remote access.

    Bridges expose robot parameters to remote clients (GCS, web UI, etc.)
    and receive parameter updates from them. They implement `BB.Bridge`.

    ## Example

        parameters do
          bridge :mavlink, {BBMavLink.ParameterBridge, conn: "/dev/ttyACM0"}
          bridge :phoenix, {BBPhoenix.ParameterBridge, url: "ws://gcs.local/socket"}
        end
    """,
    target: BB.Dsl.Bridge,
    identifier: :name,
    args: [:name, :child_spec],
    schema: [
      name: [
        type: :atom,
        required: true,
        doc: "A unique name for the bridge"
      ],
      child_spec: [
        type:
          {:or, [{:behaviour, BB.Bridge}, {:tuple, [{:behaviour, BB.Bridge}, :keyword_list]}]},
        required: true,
        doc:
          "The child specification for the bridge process. Either a module or `{module, keyword_list}`"
      ],
      simulation: [
        type: {:in, [:omit, :mock, :start]},
        default: :omit,
        doc:
          "Behaviour in simulation mode: :omit (don't start), :mock (start no-op mock), :start (start real bridge)"
      ]
    ]
  }

  @parameters %Section{
    name: :parameters,
    describe: """
    Runtime-adjustable parameters for the robot.

    Parameters provide a way to configure robot behaviour at runtime without
    recompilation. They support validation, change notifications via PubSub,
    and optional persistence.

    ## Example

        parameters do
          group :motion do
            param :max_linear_speed, type: :float, default: 1.0,
              min: 0.0, max: 10.0, doc: "Max velocity in m/s"
            param :max_angular_speed, type: :float, default: 0.5
          end

          group :safety do
            param :collision_distance, type: :float, default: 0.3
          end
        end
    """,
    entities: [@param_group, @param, @bridge]
  }

  @topology %Section{
    name: :topology,
    describe: "Robot topology",
    entities: [@link, @joint]
  }

  use Spark.Dsl.Extension,
    sections: [@topology, @settings, @sensors, @controllers, @commands, @parameters],
    transformers: [
      __MODULE__.DefaultNameTransformer,
      __MODULE__.TopologyTransformer,
      __MODULE__.SupervisorTransformer,
      __MODULE__.UniquenessTransformer,
      __MODULE__.RobotTransformer,
      __MODULE__.CommandTransformer,
      __MODULE__.ParameterTransformer
    ],
    verifiers: [
      __MODULE__.Verifiers.ValidateChildSpecs,
      __MODULE__.Verifiers.ValidateParamRefs
    ]
end
