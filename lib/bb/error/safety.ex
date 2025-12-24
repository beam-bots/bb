# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Safety do
  @moduledoc """
  Safety system error classes.

  These errors represent safety-critical violations that require immediate
  response. All safety errors have `:critical` severity and trigger
  automatic disarm when handled by the safety system.

  Safety errors should be raised when:
  - Physical limits are exceeded (position, velocity, torque)
  - Collision risk is detected
  - Emergency stop is triggered
  - Disarm callbacks fail (hardware may be in unsafe state)
  """
end
