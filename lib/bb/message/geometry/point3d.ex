# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Geometry.Point3D do
  @moduledoc """
  A 3D point in space.

  ## Fields

  - `x` - X coordinate in metres
  - `y` - Y coordinate in metres
  - `z` - Z coordinate in metres

  ## Examples

      alias BB.Message.Geometry.Point3D

      {:ok, msg} = Point3D.new(:base_link, x: 0.3, y: 0.2, z: 0.1)

      # Convert to tuple for IK solvers
      point = msg.payload
      {point.x, point.y, point.z}  # => {0.3, 0.2, 0.1}
  """

  defstruct [:x, :y, :z]

  use BB.Message,
    schema: [
      x: [type: :float, required: true, doc: "X coordinate in metres"],
      y: [type: :float, required: true, doc: "Y coordinate in metres"],
      z: [type: :float, required: true, doc: "Z coordinate in metres"]
    ]

  @type t :: %__MODULE__{
          x: float(),
          y: float(),
          z: float()
        }

  @doc """
  Convert to a plain tuple for use with IK solvers.
  """
  @spec to_tuple(t()) :: {float(), float(), float()}
  def to_tuple(%__MODULE__{x: x, y: y, z: z}), do: {x, y, z}
end
