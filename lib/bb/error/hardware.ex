# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware do
  @moduledoc """
  Hardware communication error classes.

  These errors represent failures in communication with physical devices
  such as servos, sensors, and motor controllers.

  All hardware errors have `:error` severity by default, meaning they don't
  trigger automatic disarm. Transient hardware issues (like communication
  timeouts) are common and shouldn't cause spurious safety responses.
  """
end
