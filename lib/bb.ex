# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB do
  @moduledoc """
  Documentation for `BB` (Beam Bots).
  """
  use Spark.Dsl, default_extensions: [extensions: [BB.Dsl]]

  defdelegate subscribe(module, path, opts \\ []), to: BB.PubSub
  defdelegate unsubscribe(module, path), to: BB.PubSub
  defdelegate publish(module, path, message), to: BB.PubSub

  defdelegate call(module, name, message, timeout \\ 5000), to: BB.Process
  defdelegate cast(module, name, message), to: BB.Process
  defdelegate send(module, name, message), to: BB.Process
end
