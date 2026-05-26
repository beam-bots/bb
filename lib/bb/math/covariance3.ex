# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Covariance3 do
  @moduledoc """
  A 3x3 covariance matrix, backed by an Nx tensor.

  Used to express uncertainty over a 3-vector quantity such as an
  accelerometer reading, an angular velocity, or a position. The matrix
  is mathematically expected to be symmetric and positive semi-definite;
  this module does not enforce those invariants on construction (zero
  matrices, in particular, are legitimate "unknown variance" placeholders
  and must remain constructible).

  Follows the same typed-Nx-wrapper pattern as `BB.Math.Vec3`,
  `BB.Math.Quaternion`, and `BB.Math.Transform`.

  ## Examples

      iex> c = BB.Math.Covariance3.diagonal([0.01, 0.01, 0.02])
      iex> BB.Math.Covariance3.get(c, 0, 0)
      0.01
  """

  defstruct [:tensor]

  @type t :: %__MODULE__{tensor: Nx.Tensor.t()}

  @doc """
  Creates a covariance from a `{3, 3}` tensor.

  ## Examples

      iex> tensor = Nx.tensor([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]])
      iex> c = BB.Math.Covariance3.new(tensor)
      iex> BB.Math.Covariance3.get(c, 1, 1)
      2.0
  """
  @spec new(Nx.Tensor.t()) :: t()
  def new(tensor) do
    case Nx.shape(tensor) do
      {3, 3} -> %__MODULE__{tensor: Nx.as_type(tensor, :f64)}
      shape -> raise ArgumentError, "expected a {3, 3} tensor, got: #{inspect(shape)}"
    end
  end

  @doc """
  Returns the zero covariance - all variances and correlations zero.

  ## Examples

      iex> c = BB.Math.Covariance3.zero()
      iex> BB.Math.Covariance3.get(c, 0, 0)
      0.0
  """
  @spec zero() :: t()
  def zero, do: %__MODULE__{tensor: Nx.broadcast(Nx.tensor(0.0, type: :f64), {3, 3})}

  @doc """
  Returns the 3x3 identity matrix as a covariance.

  ## Examples

      iex> c = BB.Math.Covariance3.identity()
      iex> {BB.Math.Covariance3.get(c, 0, 0), BB.Math.Covariance3.get(c, 0, 1)}
      {1.0, 0.0}
  """
  @spec identity() :: t()
  def identity, do: %__MODULE__{tensor: Nx.eye(3, type: :f64)}

  @doc """
  Builds a diagonal covariance from a list of three variances or a
  `{3}` tensor.

  ## Examples

      iex> c = BB.Math.Covariance3.diagonal([0.1, 0.2, 0.3])
      iex> {BB.Math.Covariance3.get(c, 0, 0), BB.Math.Covariance3.get(c, 1, 1), BB.Math.Covariance3.get(c, 2, 2)}
      {0.1, 0.2, 0.3}
  """
  @spec diagonal([number()] | Nx.Tensor.t()) :: t()
  def diagonal(values) when is_list(values) do
    case values do
      [_, _, _] ->
        %__MODULE__{tensor: values |> Nx.tensor(type: :f64) |> Nx.make_diagonal()}

      _ ->
        raise ArgumentError, "expected a list of three numbers, got: #{inspect(values)}"
    end
  end

  def diagonal(%Nx.Tensor{} = tensor) do
    case Nx.shape(tensor) do
      {3} -> %__MODULE__{tensor: tensor |> Nx.as_type(:f64) |> Nx.make_diagonal()}
      shape -> raise ArgumentError, "expected a {3} tensor, got: #{inspect(shape)}"
    end
  end

  @doc """
  Wraps an existing `{3, 3}` tensor without validation. Convenience
  alias for `new/1` matching the convention used by `BB.Math.Transform`.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor), do: new(tensor)

  @doc "Returns the underlying tensor."
  @spec to_tensor(t()) :: Nx.Tensor.t()
  def to_tensor(%__MODULE__{tensor: t}), do: t

  @doc """
  Reads a scalar element at row/column `(i, j)`.

  ## Examples

      iex> c = BB.Math.Covariance3.diagonal([0.5, 1.5, 2.5])
      iex> BB.Math.Covariance3.get(c, 2, 2)
      2.5
  """
  @spec get(t(), 0..2, 0..2) :: float()
  def get(%__MODULE__{tensor: t}, i, j) when i in 0..2 and j in 0..2 do
    Nx.to_number(t[i][j])
  end
end
