# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Covariance6 do
  @moduledoc """
  A 6x6 covariance matrix, backed by an Nx tensor.

  Used to express uncertainty over 6-DOF pose or twist estimates (3
  translation + 3 rotation, or 3 linear + 3 angular). The matrix is
  mathematically expected to be symmetric and positive semi-definite;
  this module does not enforce those invariants on construction.

  Follows the same typed-Nx-wrapper pattern as `BB.Math.Covariance3`.

  ## Examples

      iex> c = BB.Math.Covariance6.diagonal([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
      iex> BB.Math.Covariance6.get(c, 3, 3)
      0.001
  """

  defstruct [:tensor]

  @type t :: %__MODULE__{tensor: Nx.Tensor.t()}

  @doc """
  Creates a covariance from a `{6, 6}` tensor.
  """
  @spec new(Nx.Tensor.t()) :: t()
  def new(tensor) do
    case Nx.shape(tensor) do
      {6, 6} -> %__MODULE__{tensor: Nx.as_type(tensor, :f64)}
      shape -> raise ArgumentError, "expected a {6, 6} tensor, got: #{inspect(shape)}"
    end
  end

  @doc """
  Returns the zero covariance.

  ## Examples

      iex> c = BB.Math.Covariance6.zero()
      iex> BB.Math.Covariance6.get(c, 0, 0)
      0.0
  """
  @spec zero() :: t()
  def zero, do: %__MODULE__{tensor: Nx.broadcast(Nx.tensor(0.0, type: :f64), {6, 6})}

  @doc """
  Returns the 6x6 identity matrix as a covariance.

  ## Examples

      iex> c = BB.Math.Covariance6.identity()
      iex> {BB.Math.Covariance6.get(c, 0, 0), BB.Math.Covariance6.get(c, 0, 1)}
      {1.0, 0.0}
  """
  @spec identity() :: t()
  def identity, do: %__MODULE__{tensor: Nx.eye(6, type: :f64)}

  @doc """
  Builds a diagonal covariance from a list of six variances or a `{6}`
  tensor.

  ## Examples

      iex> c = BB.Math.Covariance6.diagonal([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
      iex> BB.Math.Covariance6.get(c, 5, 5)
      6.0
  """
  @spec diagonal([number()] | Nx.Tensor.t()) :: t()
  def diagonal(values) when is_list(values) do
    case values do
      [_, _, _, _, _, _] ->
        %__MODULE__{tensor: values |> Nx.tensor(type: :f64) |> Nx.make_diagonal()}

      _ ->
        raise ArgumentError, "expected a list of six numbers, got: #{inspect(values)}"
    end
  end

  def diagonal(%Nx.Tensor{} = tensor) do
    case Nx.shape(tensor) do
      {6} -> %__MODULE__{tensor: tensor |> Nx.as_type(:f64) |> Nx.make_diagonal()}
      shape -> raise ArgumentError, "expected a {6} tensor, got: #{inspect(shape)}"
    end
  end

  @doc """
  Wraps an existing `{6, 6}` tensor. Convenience alias for `new/1`.
  """
  @spec from_tensor(Nx.Tensor.t()) :: t()
  def from_tensor(tensor), do: new(tensor)

  @doc "Returns the underlying tensor."
  @spec to_tensor(t()) :: Nx.Tensor.t()
  def to_tensor(%__MODULE__{tensor: t}), do: t

  @doc """
  Reads a scalar element at row/column `(i, j)`.
  """
  @spec get(t(), 0..5, 0..5) :: float()
  def get(%__MODULE__{tensor: t}, i, j) when i in 0..5 and j in 0..5 do
    Nx.to_number(t[i][j])
  end
end
