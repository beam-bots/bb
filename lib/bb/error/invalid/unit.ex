# SPDX-FileCopyrightText: 2026 James Harton <james.harton@alembic.com.au>
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Unit do
  @moduledoc """
  Invalid unit identifier or incompatible unit conversion.

  Raised when an identifier does not name a unit `BB.Unit.Conversions` can
  convert, or when two units are compared or converted across dimensional
  categories.
  """
  use BB.Error,
    class: :invalid,
    fields: [:unit, :expected]

  @type t :: %__MODULE__{
          unit: String.t(),
          expected: String.t() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{unit: unit, expected: nil}) do
    "`#{unit}` is not a known unit"
  end

  def message(%{unit: unit, expected: expected}) do
    "The unit `#{unit}` is not compatible with `#{expected}`"
  end
end
