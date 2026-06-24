# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Bridge do
  @moduledoc """
  Parameter-bridge validation errors.

  These errors are raised by `BB.Bridge` implementations when a parameter
  read or write is rejected — an unknown or read-only parameter, a malformed
  parameter id, or an attempt to modify a parameter while torque is enabled.

  They live in `bb` core so every bridge driver shares one set of error
  modules rather than each redefining them in the `BB.Error.*` namespace.
  """
end
