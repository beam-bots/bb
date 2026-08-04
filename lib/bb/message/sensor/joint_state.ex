# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Sensor.JointState do
  @moduledoc """
  State of a set of joints.

  ## Fields

  - `names` - List of joint names as atoms
  - `positions` - Joint configurations
  - `velocities` - Joint velocities
  - `efforts` - Joint efforts

  All lists must have the same length. Missing values can be represented
  as empty lists.

  ## Values are shaped to the joint's type

  A joint with more than one degree of freedom has no single number to report, so
  each list is heterogeneous and its elements line up with `names`:

  | Joint type | `positions` | `velocities` | `efforts` |
  |---|---|---|---|
  | single-DoF | `float` | `float` | `float` |
  | `:planar` | `BB.Math.Transform2D` | `BB.Message.Geometry.Twist2D` | `BB.Message.Geometry.Wrench2D` |
  | `:floating` | `BB.Math.Transform` | `BB.Message.Geometry.Twist` | `BB.Message.Geometry.Wrench` |

  Most consumers only ever see single-DoF joints, so in practice their handling is
  unchanged — but a consumer that pattern matches on a float, or does arithmetic
  on an element, should say which joint types it supports.

  Splitting multi-DoF state into a separate message type was considered and
  rejected: a robot's joint state would then arrive on two topics, and every
  consumer wanting the whole configuration would have to subscribe to both and
  correlate them by timestamp to reconstruct one instant. That is a permanent tax
  on every consumer, levied to avoid changing one message.

  All three lists changed together for the same reason. `velocities` and `efforts`
  have precisely the same problem as `positions`, and breaking a widely-consumed
  message twice is worse than breaking it once.

  ## Examples

      alias BB.Message.Sensor.JointState

      {:ok, msg} = JointState.new(:arm,
        names: [:joint1, :joint2],
        positions: [0.0, 1.57],
        velocities: [0.1, 0.0],
        efforts: [0.5, 0.2]
      )

      # A mobile base reports its whole pose in the same message as its mast
      {:ok, msg} = JointState.new(:rover,
        names: [:base, :mast],
        positions: [BB.Math.Transform2D.new(12.4, -3.1, 1.57), 0.5]
      )
  """

  import BB.Message.Option

  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Message.Geometry.Twist
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Geometry.Wrench
  alias BB.Message.Geometry.Wrench2D

  defstruct [:names, :positions, :velocities, :efforts]

  use BB.Message,
    schema: [
      names: [type: {:list, :atom}, required: true, doc: "Joint names"],
      positions: [
        type: configurations_type(),
        default: [],
        doc: "Joint configurations, shaped to each joint's type"
      ],
      velocities: [
        type: velocities_type(),
        default: [],
        doc: "Joint velocities, shaped to each joint's type"
      ],
      efforts: [
        type: efforts_type(),
        default: [],
        doc: "Joint efforts, shaped to each joint's type"
      ]
    ]

  @typedoc "A joint's configuration, shaped to its type."
  @type configuration :: float() | Transform2D.t() | Transform.t()

  @typedoc "A joint's velocity, shaped to its type."
  @type velocity :: float() | Twist2D.t() | Twist.t()

  @typedoc "A joint's effort, shaped to its type."
  @type effort :: float() | Wrench2D.t() | Wrench.t()

  @type t :: %__MODULE__{
          names: [atom()],
          positions: [configuration()],
          velocities: [velocity()],
          efforts: [effort()]
        }
end
