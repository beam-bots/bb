# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Wrench2D do
  @moduledoc """
  Force and torque within a plane.

  The planar counterpart of `BB.Message.Geometry.Wrench`, and the effort of a
  `:planar` joint. `fx`/`fy` are in the plane spanned by the joint's `axis`
  normal, and `tau` is about that normal — the same basis
  `BB.Math.Transform2D.to_transform/2` uses.

  Using the 3D `Wrench` with the out-of-plane components zeroed would let a
  producer emit a physically impossible out-of-plane force that nothing would
  reject.

  ## Fields

  - `fx` - In-plane force along the plane's first axis, in Newtons
  - `fy` - In-plane force along the plane's second axis, in Newtons
  - `tau` - Torque about the plane's normal, in Newton-metres

  ## Examples

      alias BB.Message.Geometry.Wrench2D

      {:ok, msg} = Wrench2D.new(:chassis, 12.0, 0.0, 0.4)
  """

  defstruct [:fx, :fy, :tau]

  use BB.Message,
    schema: [
      fx: [type: :float, required: true, doc: "In-plane force in Newtons"],
      fy: [type: :float, required: true, doc: "In-plane force in Newtons"],
      tau: [type: :float, required: true, doc: "Torque about the normal in Newton-metres"]
    ]

  @type t :: %__MODULE__{fx: float(), fy: float(), tau: float()}

  @doc """
  Create a new Wrench2D message.

  Returns `{:ok, %BB.Message{}}` with the wrench as payload.

  ## Examples

      {:ok, msg} = Wrench2D.new(:chassis, 12.0, 0.0, 0.4)
  """
  @spec new(atom(), number(), number(), number()) :: {:ok, BB.Message.t()} | {:error, term()}
  def new(frame_id, fx, fy, tau) when is_number(fx) and is_number(fy) and is_number(tau) do
    new(frame_id, fx: fx / 1, fy: fy / 1, tau: tau / 1)
  end
end
