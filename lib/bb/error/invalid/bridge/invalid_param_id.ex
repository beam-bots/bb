# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Bridge.InvalidParamId do
  @moduledoc """
  Invalid parameter ID format for bridge parameter.

  The parameter ID must be in the format `"servo_id:param_name"` where
  `servo_id` is an integer between 1 and 253.
  """
  use BB.Error,
    class: :invalid,
    fields: [:param_id]

  @type t :: %__MODULE__{
          param_id: term()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{param_id: param_id}) do
    "Invalid parameter ID format: #{inspect(param_id)}. " <>
      "Expected format: \"servo_id:param_name\" (e.g., \"1:position_p_gain\")"
  end
end
