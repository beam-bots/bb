# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defprotocol Kinetix.Message.Payload do
  @moduledoc """
  Protocol for introspecting message payloads.

  Payload types implement this protocol to enable runtime introspection
  of their schema.
  """

  @doc "Returns the Spark.Options schema for this payload type"
  @spec schema(t) :: Spark.Options.t()
  def schema(payload)
end
