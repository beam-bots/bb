# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule TestLogFilter do
  @moduledoc false

  @doc false
  def log(%{meta: %{mfa: {BB.Safety.Controller, _, _}}}, _), do: :stop
  def log(_, _), do: :ignore
end
