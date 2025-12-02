defmodule Kinetix.Cldr do
  @moduledoc """
  Kinetix uses CLDR to manage unit conversions.
  """

  use Cldr,
    locales: ["en"],
    default_locale: "en",
    providers: [Cldr.Number, Cldr.Unit]
end
