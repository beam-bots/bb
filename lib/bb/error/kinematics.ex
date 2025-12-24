# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics do
  @moduledoc """
  Kinematics and motion planning error classes.

  These errors represent failures in computing robot motion, including
  inverse kinematics failures, unreachable targets, and singularity
  conditions.

  Kinematics errors have `:error` severity - they indicate the requested
  motion cannot be achieved, but don't represent a safety hazard.
  """
end
