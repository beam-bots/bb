# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Application do
  @moduledoc false
  use Application

  @impl Application
  def start(_type, _args) do
    [
      BB.Safety.Controller,
      BB.Command.ResultCache
    ]
    |> Supervisor.start_link(strategy: :one_for_one, name: BB.Supervisor.Application)
  end
end
