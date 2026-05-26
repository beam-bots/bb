# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Covariance3Test do
  use ExUnit.Case, async: true
  doctest BB.Math.Covariance3

  alias BB.Math.Covariance3

  describe "new/1" do
    test "wraps a {3, 3} tensor" do
      tensor = Nx.tensor([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]])
      cov = Covariance3.new(tensor)
      assert Covariance3.get(cov, 1, 1) == 2.0
    end

    test "rejects non-3x3 tensors" do
      assert_raise ArgumentError, ~r/expected a \{3, 3\} tensor/, fn ->
        Covariance3.new(Nx.tensor([[1.0]]))
      end
    end
  end

  describe "zero/0" do
    test "returns a 3x3 zero matrix" do
      cov = Covariance3.zero()

      for i <- 0..2, j <- 0..2 do
        assert Covariance3.get(cov, i, j) == 0.0
      end
    end
  end

  describe "identity/0" do
    test "is 1 on the diagonal, 0 off-diagonal" do
      cov = Covariance3.identity()
      assert Covariance3.get(cov, 0, 0) == 1.0
      assert Covariance3.get(cov, 1, 1) == 1.0
      assert Covariance3.get(cov, 2, 2) == 1.0
      assert Covariance3.get(cov, 0, 1) == 0.0
      assert Covariance3.get(cov, 2, 0) == 0.0
    end
  end

  describe "diagonal/1" do
    test "from a list of three numbers" do
      cov = Covariance3.diagonal([0.1, 0.2, 0.3])

      assert Covariance3.get(cov, 0, 0) == 0.1
      assert Covariance3.get(cov, 1, 1) == 0.2
      assert Covariance3.get(cov, 2, 2) == 0.3
      assert Covariance3.get(cov, 0, 1) == 0.0
    end

    test "from a {3} tensor" do
      cov = Covariance3.diagonal(Nx.tensor([0.5, 1.5, 2.5], type: :f64))
      assert Covariance3.get(cov, 1, 1) == 1.5
    end

    test "rejects non-three lists" do
      assert_raise ArgumentError, fn -> Covariance3.diagonal([1.0]) end
      assert_raise ArgumentError, fn -> Covariance3.diagonal([1.0, 2.0]) end
      assert_raise ArgumentError, fn -> Covariance3.diagonal([1.0, 2.0, 3.0, 4.0]) end
    end
  end

  describe "to_tensor/1" do
    test "round-trips through new/1" do
      original = Nx.tensor([[1.0, 0.5, 0.0], [0.5, 2.0, 0.0], [0.0, 0.0, 3.0]])
      cov = Covariance3.new(original)
      assert Nx.to_flat_list(Covariance3.to_tensor(cov)) == Nx.to_flat_list(original)
    end
  end
end
