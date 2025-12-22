# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ParamRef do
  @moduledoc """
  A reference to a parameter for use in DSL fields.

  Instead of providing a literal unit value in the DSL, users can reference
  a parameter that will be resolved at runtime. This enables runtime-adjustable
  configuration for values that would otherwise be compile-time constants.

  ## Usage

  ```elixir
  parameters do
    group :motion do
      param :max_effort, type: {:unit, :newton_meter}, default: ~u(10 newton_meter)
    end
  end

  topology do
    link :base do
      joint :shoulder do
        limit do
          effort(param([:motion, :max_effort]))
        end
      end
    end
  end
  ```

  The `param/1` function creates a reference that:
  - Is validated at compile-time to ensure the parameter exists
  - Is resolved at robot startup to get the current parameter value
  - Subscribes to parameter changes to keep the robot struct updated
  """

  defstruct [:path, :expected_unit_type]

  @type t :: %__MODULE__{
          path: [atom()],
          expected_unit_type: atom() | nil
        }

  @doc """
  Create a parameter reference for DSL fields.

  The path should match a parameter defined in the `parameters` section of
  the robot DSL.

  ## Examples

      param([:motion, :max_speed])
      param([:limits, :shoulder, :effort])
  """
  @spec param([atom()]) :: t()
  def param([_ | _] = path) do
    %__MODULE__{path: path}
  end
end
