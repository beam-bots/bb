# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Application do
  @moduledoc false
  use Application

  @impl Application
  def start(_type, _args) do
    children = [
      BB.Safety.Controller
    ]

    opts = [strategy: :one_for_one, name: BB.Supervisor.Application]
    Supervisor.start_link(children, opts)
  end
end
