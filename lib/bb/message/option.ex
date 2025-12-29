# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Message.Option do
  @moduledoc """
  Custom Spark.Options types for message primitives.

  Provides type functions for use in payload schemas to validate
  `BB.Math.Vec3.t()`, `BB.Math.Quaternion.t()`, and `BB.Math.Transform.t()` types.

  ## Usage

      import BB.Message.Option

      @schema Spark.Options.new!([
        position: [type: vec3_type(), required: true],
        orientation: [type: quaternion_type(), required: true],
        pose: [type: transform_type(), required: true]
      ])
  """

  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3

  @doc """
  Returns a Spark.Options type for validating `BB.Vec3.t()`.

  ## Examples

      iex> BB.Message.Option.vec3_type()
      {:custom, BB.Message.Option, :validate_vec3, [[]]}
  """
  @spec vec3_type() :: {:custom, module(), atom(), list()}
  def vec3_type, do: {:custom, __MODULE__, :validate_vec3, [[]]}

  @doc """
  Returns a Spark.Options type for validating `BB.Quaternion.t()`.

  ## Examples

      iex> BB.Message.Option.quaternion_type()
      {:custom, BB.Message.Option, :validate_quaternion, [[]]}
  """
  @spec quaternion_type() :: {:custom, module(), atom(), list()}
  def quaternion_type, do: {:custom, __MODULE__, :validate_quaternion, [[]]}

  @doc """
  Validates a BB.Vec3 struct.

  ## Examples

      iex> BB.Message.Option.validate_vec3(BB.Vec3.new(1.0, 2.0, 3.0), [])
      {:ok, %BB.Vec3{}}

      iex> BB.Message.Option.validate_vec3("not a vec3", [])
      {:error, "expected BB.Vec3.t(), got: \\"not a vec3\\""}
  """
  @spec validate_vec3(term(), keyword()) :: {:ok, Vec3.t()} | {:error, String.t()}
  def validate_vec3(%Vec3{} = vec, _opts), do: {:ok, vec}

  def validate_vec3(value, _opts) do
    {:error, "expected BB.Vec3.t(), got: #{inspect(value)}"}
  end

  @doc """
  Validates a BB.Quaternion struct.

  ## Examples

      iex> BB.Message.Option.validate_quaternion(BB.Quaternion.identity(), [])
      {:ok, %BB.Quaternion{}}

      iex> BB.Message.Option.validate_quaternion("not a quaternion", [])
      {:error, "expected BB.Quaternion.t(), got: \\"not a quaternion\\""}
  """
  @spec validate_quaternion(term(), keyword()) :: {:ok, Quaternion.t()} | {:error, String.t()}
  def validate_quaternion(%Quaternion{} = quat, _opts), do: {:ok, quat}

  def validate_quaternion(value, _opts) do
    {:error, "expected BB.Quaternion.t(), got: #{inspect(value)}"}
  end

  @doc """
  Returns a Spark.Options type for validating `BB.Math.Transform.t()`.

  ## Examples

      iex> BB.Message.Option.transform_type()
      {:custom, BB.Message.Option, :validate_transform, [[]]}
  """
  @spec transform_type() :: {:custom, module(), atom(), list()}
  def transform_type, do: {:custom, __MODULE__, :validate_transform, [[]]}

  @doc """
  Validates a BB.Math.Transform struct.

  ## Examples

      iex> BB.Message.Option.validate_transform(BB.Math.Transform.identity(), [])
      {:ok, %BB.Math.Transform{}}

      iex> BB.Message.Option.validate_transform("not a transform", [])
      {:error, "expected BB.Math.Transform.t(), got: \\"not a transform\\""}
  """
  @spec validate_transform(term(), keyword()) :: {:ok, Transform.t()} | {:error, String.t()}
  def validate_transform(%Transform{} = transform, _opts), do: {:ok, transform}

  def validate_transform(value, _opts) do
    {:error, "expected BB.Math.Transform.t(), got: #{inspect(value)}"}
  end
end
