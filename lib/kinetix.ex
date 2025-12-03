# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Kinetix do
  @moduledoc """
  Documentation for `Kinetix`.
  """
  use Spark.Dsl, default_extensions: [extensions: [Kinetix.Dsl]]

  defdelegate subscribe(module, path, opts \\ []), to: Kinetix.PubSub
  defdelegate unsubscribe(module, path), to: Kinetix.PubSub
  defdelegate publish(module, path, message), to: Kinetix.PubSub

  defdelegate call(module, name, message, timeout \\ 5000), to: Kinetix.Process
  defdelegate cast(module, name, message), to: Kinetix.Process
  defdelegate send(module, name, message), to: Kinetix.Process
end
