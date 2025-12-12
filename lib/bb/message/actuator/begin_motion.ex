# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Actuator.BeginMotion do
  @moduledoc """
  Message published by actuators when beginning a motion.

  Used by `BB.Sensor.OpenLoopPositionEstimator` to estimate current position
  during open-loop control (actuators without position feedback).

  ## Fields

  - `initial_position` - Position before motion begins (radians or metres)
  - `target_position` - Target position (radians or metres)
  - `expected_arrival` - When motion should complete (monotonic milliseconds)

  ## Example

      alias BB.Message
      alias BB.Message.Actuator.BeginMotion

      expected_arrival = System.monotonic_time(:millisecond) + 500

      {:ok, msg} = Message.new(BeginMotion, :shoulder,
        initial_position: 0.0,
        target_position: 1.57,
        expected_arrival: expected_arrival
      )
  """

  defstruct [:initial_position, :target_position, :expected_arrival]

  use BB.Message,
    schema: [
      initial_position: [
        type: :float,
        required: true,
        doc: "Starting position (radians or metres)"
      ],
      target_position: [type: :float, required: true, doc: "Target position (radians or metres)"],
      expected_arrival: [
        type: :integer,
        required: true,
        doc: "Expected arrival time (monotonic milliseconds)"
      ]
    ]

  @type t :: %__MODULE__{
          initial_position: float(),
          target_position: float(),
          expected_arrival: integer()
        }
end
