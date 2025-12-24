# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.FailingActuator do
  @moduledoc false
  use BB.Actuator, options_schema: [fail_mode: [type: :atom, required: false]]

  @impl BB.Actuator
  def disarm(opts) do
    case opts[:fail_mode] do
      :error -> {:error, :hardware_failure}
      :raise -> raise "Hardware communication failed"
      :throw -> throw(:hardware_timeout)
      :slow -> Process.sleep(10_000)
      _ -> :ok
    end
  end

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)

    BB.Safety.register(__MODULE__,
      robot: bb.robot,
      path: bb.path,
      opts: [fail_mode: opts[:fail_mode]]
    )

    {:ok, %{}}
  end
end
