# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Test.FailingActuator do
  @moduledoc false
  use GenServer
  @behaviour BB.Safety

  @impl BB.Safety
  def disarm(opts) do
    case opts[:fail_mode] do
      :error -> {:error, :hardware_failure}
      :raise -> raise "Hardware communication failed"
      :throw -> throw(:hardware_timeout)
      :slow -> Process.sleep(10_000)
      _ -> :ok
    end
  end

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts)
  end

  @impl GenServer
  def init(opts) do
    BB.Safety.register(__MODULE__,
      robot: opts[:bb].robot,
      path: opts[:bb].path,
      opts: [fail_mode: opts[:fail_mode]]
    )

    {:ok, %{}}
  end
end
