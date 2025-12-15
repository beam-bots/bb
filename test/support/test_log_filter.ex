defmodule TestLogFilter do
  def log(%{meta: %{mfa: {BB.Safety.Controller, _, _}}}, _), do: :stop
  def log(_, _), do: :ignore
end
