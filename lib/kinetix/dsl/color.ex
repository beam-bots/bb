defmodule Kinetix.Dsl.Color do
  @moduledoc """
  A color
  """
  defstruct __identifier__: nil,
            __spark_metadata__: nil,
            red: nil,
            green: nil,
            blue: nil,
            alpha: nil

  alias Spark.Dsl.Entity

  @type t :: %__MODULE__{
          __identifier__: any,
          __spark_metadata__: Entity.spark_meta(),
          red: number,
          green: number,
          blue: number,
          alpha: number
        }

  @doc "Validate a color channel value (must be between 0 and 1)"
  @spec validate(any) :: {:ok, number} | {:error, String.t()}
  def validate(value) when is_number(value) and value >= 0 and value <= 1, do: {:ok, value}

  def validate(value) when is_number(value),
    do: {:error, "Color value must be between 0 and 1, got: #{value}"}

  def validate(value), do: {:error, "Expected a number for color value, got: #{inspect(value)}"}
end
