# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Twist2D do
  @moduledoc """
  Linear and angular velocity within a plane.

  The planar counterpart of `BB.Message.Geometry.Twist`, and the velocity of a
  `:planar` joint. `vx`/`vy` are in the plane spanned by the joint's `axis`
  normal, and `omega` is about that normal — the same basis
  `BB.Math.Transform2D.to_transform/2` uses.

  Using the 3D `Twist` with the out-of-plane components zeroed would let a
  producer emit a physically impossible out-of-plane velocity that nothing would
  reject.

  ## Fields

  - `vx` - In-plane linear velocity along the plane's first axis, in m/s
  - `vy` - In-plane linear velocity along the plane's second axis, in m/s
  - `omega` - Angular velocity about the plane's normal, in rad/s

  ## Examples

      alias BB.Message.Geometry.Twist2D

      {:ok, msg} = Twist2D.new(:chassis, 0.5, 0.0, 0.1)
  """

  defstruct [:vx, :vy, :omega]

  use BB.Message,
    schema: [
      vx: [type: :float, required: true, doc: "In-plane linear velocity in m/s"],
      vy: [type: :float, required: true, doc: "In-plane linear velocity in m/s"],
      omega: [type: :float, required: true, doc: "Angular velocity about the normal in rad/s"]
    ]

  @type t :: %__MODULE__{vx: float(), vy: float(), omega: float()}

  @doc """
  Create a new Twist2D message.

  Returns `{:ok, %BB.Message{}}` with the twist as payload.

  ## Examples

      {:ok, msg} = Twist2D.new(:chassis, 0.5, 0.0, 0.1)
  """
  @spec new(atom(), number(), number(), number()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, vx, vy, omega) when is_number(vx) and is_number(vy) and is_number(omega) do
    new(frame_id, vx: vx / 1, vy: vy / 1, omega: omega / 1)
  end
end
