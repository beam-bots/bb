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

  alias BB.Math.Covariance3
  alias BB.Math.Covariance6
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Message.Geometry.Twist
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Geometry.Wrench
  alias BB.Message.Geometry.Wrench2D

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

  @doc """
  Returns a Spark.Options type for validating `BB.Math.Covariance3.t()`.

  ## Examples

      iex> BB.Message.Option.covariance3_type()
      {:custom, BB.Message.Option, :validate_covariance3, [[]]}
  """
  @spec covariance3_type() :: {:custom, module(), atom(), list()}
  def covariance3_type, do: {:custom, __MODULE__, :validate_covariance3, [[]]}

  @doc """
  Validates a BB.Math.Covariance3 struct.
  """
  @spec validate_covariance3(term(), keyword()) ::
          {:ok, Covariance3.t()} | {:error, String.t()}
  def validate_covariance3(%Covariance3{} = cov, _opts), do: {:ok, cov}

  def validate_covariance3(value, _opts) do
    {:error, "expected BB.Math.Covariance3.t(), got: #{inspect(value)}"}
  end

  @doc """
  Returns a Spark.Options type for validating `BB.Math.Covariance6.t()`.

  ## Examples

      iex> BB.Message.Option.covariance6_type()
      {:custom, BB.Message.Option, :validate_covariance6, [[]]}
  """
  @spec covariance6_type() :: {:custom, module(), atom(), list()}
  def covariance6_type, do: {:custom, __MODULE__, :validate_covariance6, [[]]}

  @doc """
  Validates a BB.Math.Covariance6 struct.
  """
  @spec validate_covariance6(term(), keyword()) ::
          {:ok, Covariance6.t()} | {:error, String.t()}
  def validate_covariance6(%Covariance6{} = cov, _opts), do: {:ok, cov}

  def validate_covariance6(value, _opts) do
    {:error, "expected BB.Math.Covariance6.t(), got: #{inspect(value)}"}
  end

  @doc """
  Returns a Spark.Options type for a list of joint configurations.

  A joint's configuration is shaped to its type, so the list is heterogeneous: a
  float for single-DoF joints, a `BB.Math.Transform2D` for `:planar` and a
  `BB.Math.Transform` for `:floating`.

  This checks each element is one of those three, which is as much as a message
  can check — matching a *particular* joint's type needs the robot, which the
  message does not carry. `BB.Robot.State.set_configuration/3` is what rejects a
  shape that doesn't match its joint.

  ## Examples

      iex> BB.Message.Option.configurations_type()
      {:custom, BB.Message.Option, :validate_configurations, [[]]}
  """
  @spec configurations_type() :: {:custom, module(), atom(), list()}
  def configurations_type, do: {:custom, __MODULE__, :validate_configurations, [[]]}

  @doc """
  Returns a Spark.Options type for a list of joint velocities.

  As `configurations_type/0`, but the multi-DoF shapes are
  `BB.Message.Geometry.Twist2D` and `BB.Message.Geometry.Twist`.

  ## Examples

      iex> BB.Message.Option.velocities_type()
      {:custom, BB.Message.Option, :validate_velocities, [[]]}
  """
  @spec velocities_type() :: {:custom, module(), atom(), list()}
  def velocities_type, do: {:custom, __MODULE__, :validate_velocities, [[]]}

  @doc """
  Returns a Spark.Options type for a list of joint efforts.

  As `configurations_type/0`, but the multi-DoF shapes are
  `BB.Message.Geometry.Wrench2D` and `BB.Message.Geometry.Wrench`.

  ## Examples

      iex> BB.Message.Option.efforts_type()
      {:custom, BB.Message.Option, :validate_efforts, [[]]}
  """
  @spec efforts_type() :: {:custom, module(), atom(), list()}
  def efforts_type, do: {:custom, __MODULE__, :validate_efforts, [[]]}

  @doc """
  Validates a list of joint configurations.

  ## Examples

      iex> BB.Message.Option.validate_configurations([0.5, BB.Math.Transform.identity()], [])
      {:ok, [0.5, BB.Math.Transform.identity()]}

      iex> BB.Message.Option.validate_configurations([:nope], [])
      {:error,
       "expected element 0 to be a float, BB.Math.Transform2D.t() or BB.Math.Transform.t(), got: :nope"}
  """
  @spec validate_configurations(term(), keyword()) :: {:ok, list()} | {:error, String.t()}
  def validate_configurations(values, _opts) do
    validate_joint_values(
      values,
      &configuration?/1,
      "a float, BB.Math.Transform2D.t() or BB.Math.Transform.t()"
    )
  end

  @doc "Validates a list of joint velocities."
  @spec validate_velocities(term(), keyword()) :: {:ok, list()} | {:error, String.t()}
  def validate_velocities(values, _opts) do
    validate_joint_values(
      values,
      &velocity?/1,
      "a float, BB.Message.Geometry.Twist2D.t() or BB.Message.Geometry.Twist.t()"
    )
  end

  @doc "Validates a list of joint efforts."
  @spec validate_efforts(term(), keyword()) :: {:ok, list()} | {:error, String.t()}
  def validate_efforts(values, _opts) do
    validate_joint_values(
      values,
      &effort?/1,
      "a float, BB.Message.Geometry.Wrench2D.t() or BB.Message.Geometry.Wrench.t()"
    )
  end

  # Naming the offending index matters: these lists run parallel to `names`, and
  # "element 4 is wrong" is the difference between finding the bad joint and
  # eyeballing a list of transforms.
  defp validate_joint_values(values, acceptable?, expected) when is_list(values) do
    case Enum.find_index(values, &(not acceptable?.(&1))) do
      nil ->
        {:ok, values}

      index ->
        {:error,
         "expected element #{index} to be #{expected}, got: #{inspect(Enum.at(values, index))}"}
    end
  end

  defp validate_joint_values(values, _acceptable?, _expected) do
    {:error, "expected a list, got: #{inspect(values)}"}
  end

  defp configuration?(value) when is_float(value), do: true
  defp configuration?(%Transform2D{}), do: true
  defp configuration?(%Transform{}), do: true
  defp configuration?(_), do: false

  # `is_struct/2` rather than a `%Twist{}` pattern: the message modules below
  # `BB.Message.Geometry` import this one, so matching their structs here would
  # close a compile-time cycle. Elixir 1.20 tolerates it, 1.19 deadlocks the
  # parallel compiler. A module in a guard is only a runtime reference.
  defp velocity?(value) when is_float(value), do: true
  defp velocity?(value) when is_struct(value, Twist2D), do: true
  defp velocity?(value) when is_struct(value, Twist), do: true
  defp velocity?(_), do: false

  defp effort?(value) when is_float(value), do: true
  defp effort?(value) when is_struct(value, Wrench2D), do: true
  defp effort?(value) when is_struct(value, Wrench), do: true
  defp effort?(_), do: false
end
