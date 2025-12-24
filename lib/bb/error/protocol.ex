# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol do
  @moduledoc """
  Low-level protocol error classes.

  This namespace is for errors that wrap specific protocol errors from
  device communication layers. Protocol-specific errors should be defined
  in their respective packages:

  - Robotis/Dynamixel errors → `bb_servo_robotis`
  - I2C errors → `bb_servo_pca9685` or similar

  These packages can define errors under this namespace, e.g.:

      defmodule BB.Error.Protocol.Robotis.HardwareAlert do
        use BB.Error, class: :protocol, fields: [:servo_id, :alerts]
        # ...
      end
  """
end
