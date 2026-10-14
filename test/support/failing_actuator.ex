# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.FailingActuator do
  @moduledoc false
  use BB.Actuator, options_schema: [fail_mode: [type: :atom, required: false]]

  @impl BB.Actuator
  def capabilities(_opts), do: [:position_feedback]

  @impl BB.Actuator
  def disarm(opts) do
    case opts[:fail_mode] do
      :error -> {:error, :hardware_failure}
      :raise -> raise "Hardware communication failed"
      :throw -> throw(:hardware_timeout)
      :hang -> Process.sleep(:infinity)
      :gated -> await_release(opts[:robot])
      _ -> :ok
    end
  end

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)

    BB.Safety.register(__MODULE__,
      robot: bb.robot,
      path: bb.path,
      opts: [fail_mode: opts[:fail_mode], robot: bb.robot]
    )

    {:ok, %{}}
  end

  @impl BB.Actuator
  def handle_command(_message, state), do: {:noreply, state}

  # Holds the disarm callback open until the test process registered under
  # `{BB.Test.FailingActuator, robot}` in `:persistent_term` sends `:release`.
  defp await_release(robot) do
    send(:persistent_term.get({__MODULE__, robot}), {:disarming, self()})

    receive do
      :release -> :ok
    end
  end
end
