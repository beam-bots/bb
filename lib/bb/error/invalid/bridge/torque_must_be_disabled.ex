# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Bridge.TorqueMustBeDisabled do
  @moduledoc """
  Parameter modification requires torque to be disabled.

  EEPROM parameters in serial bus servos can only be modified when torque
  is disabled. Disable torque before modifying this parameter.
  """
  use BB.Error,
    class: :invalid,
    fields: [:param_name, :servo_id]

  @type t :: %__MODULE__{
          param_name: atom(),
          servo_id: non_neg_integer() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{param_name: param_name, servo_id: nil}) do
    "Parameter #{inspect(param_name)} requires torque to be disabled before modification"
  end

  def message(%{param_name: param_name, servo_id: servo_id}) do
    "Parameter #{inspect(param_name)} on servo #{servo_id} requires torque to be disabled before modification"
  end
end
