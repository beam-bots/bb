defmodule Kinetix.Dsl.Info do
  @moduledoc false
  use Spark.InfoGenerator, extension: Kinetix.Dsl, sections: [:robot]
end
