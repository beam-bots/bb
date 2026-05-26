# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Estimator do
  @moduledoc """
  State-estimation error classes.

  Raised by estimator implementations or by `BB.Estimator.Server` when an
  input violates a constraint (too stale, out of sync with the driver
  input, missing a covariance that the algorithm requires).
  """
end
