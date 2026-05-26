# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Estimator.MissingCovariance do
  @moduledoc """
  An algorithm required a covariance field on an input payload but the
  upstream publisher left it `nil`.

  Many Kalman-family algorithms cannot proceed without measurement
  noise. They may either fall back to a configured default or raise
  this error, depending on configuration.
  """

  use BB.Error, class: :invalid, fields: [:estimator, :field]

  @type t :: %__MODULE__{
          estimator: atom(),
          field: atom()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{estimator: estimator, field: field}) do
    "estimator #{inspect(estimator)} requires #{inspect(field)} but the input did not supply it"
  end
end
