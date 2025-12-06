# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Option do
  @moduledoc """
  Custom Spark.Options types for message primitives.

  Provides type functions for use in payload schemas to validate
  tagged tuple primitives like `{:vec3, x, y, z}` and `{:quaternion, x, y, z, w}`.

  ## Usage

      import BB.Message.Option

      @schema Spark.Options.new!([
        position: [type: vec3_type(), required: true],
        orientation: [type: quaternion_type(), required: true]
      ])
  """

  @doc """
  Returns a Spark.Options type for validating `{:vec3, x, y, z}` tuples.

  ## Examples

      iex> BB.Message.Option.vec3_type()
      {:custom, BB.Message.Option, :validate_vec3, [[]]}
  """
  @spec vec3_type() :: {:custom, module(), atom(), list()}
  def vec3_type, do: {:custom, __MODULE__, :validate_vec3, [[]]}

  @doc """
  Returns a Spark.Options type for validating `{:quaternion, x, y, z, w}` tuples.

  ## Examples

      iex> BB.Message.Option.quaternion_type()
      {:custom, BB.Message.Option, :validate_quaternion, [[]]}
  """
  @spec quaternion_type() :: {:custom, module(), atom(), list()}
  def quaternion_type, do: {:custom, __MODULE__, :validate_quaternion, [[]]}

  @doc """
  Validates a vec3 tagged tuple.

  ## Examples

      iex> BB.Message.Option.validate_vec3({:vec3, 1.0, 2.0, 3.0}, [])
      {:ok, {:vec3, 1.0, 2.0, 3.0}}

      iex> BB.Message.Option.validate_vec3({:vec3, 1, 2, 3}, [])
      {:error, "expected {:vec3, x, y, z} with float values, got: {:vec3, 1, 2, 3}"}

      iex> BB.Message.Option.validate_vec3("not a vec3", [])
      {:error, "expected {:vec3, x, y, z} with float values, got: \\"not a vec3\\""}
  """
  @spec validate_vec3(term(), keyword()) :: {:ok, tuple()} | {:error, String.t()}
  def validate_vec3({:vec3, x, y, z}, _opts)
      when is_float(x) and is_float(y) and is_float(z) do
    {:ok, {:vec3, x, y, z}}
  end

  def validate_vec3(value, _opts) do
    {:error, "expected {:vec3, x, y, z} with float values, got: #{inspect(value)}"}
  end

  @doc """
  Validates a quaternion tagged tuple.

  ## Examples

      iex> BB.Message.Option.validate_quaternion({:quaternion, 0.0, 0.0, 0.0, 1.0}, [])
      {:ok, {:quaternion, 0.0, 0.0, 0.0, 1.0}}

      iex> BB.Message.Option.validate_quaternion({:quaternion, 0, 0, 0, 1}, [])
      {:error, "expected {:quaternion, x, y, z, w} with float values, got: {:quaternion, 0, 0, 0, 1}"}
  """
  @spec validate_quaternion(term(), keyword()) :: {:ok, tuple()} | {:error, String.t()}
  def validate_quaternion({:quaternion, x, y, z, w}, _opts)
      when is_float(x) and is_float(y) and is_float(z) and is_float(w) do
    {:ok, {:quaternion, x, y, z, w}}
  end

  def validate_quaternion(value, _opts) do
    {:error, "expected {:quaternion, x, y, z, w} with float values, got: #{inspect(value)}"}
  end
end
