# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Math.Covariance6Test do
  use ExUnit.Case, async: true
  doctest BB.Math.Covariance6

  alias BB.Math.Covariance6

  describe "new/1" do
    test "wraps a {6, 6} tensor" do
      tensor = Nx.broadcast(Nx.tensor(0.0, type: :f64), {6, 6})
      cov = Covariance6.new(tensor)
      assert Covariance6.get(cov, 0, 0) == 0.0
    end

    test "rejects non-6x6 tensors" do
      assert_raise ArgumentError, ~r/expected a \{6, 6\} tensor/, fn ->
        Covariance6.new(Nx.tensor([[1.0]]))
      end
    end
  end

  describe "identity/0" do
    test "is 1 on diagonal, 0 off-diagonal" do
      cov = Covariance6.identity()

      for i <- 0..5 do
        assert Covariance6.get(cov, i, i) == 1.0
      end

      assert Covariance6.get(cov, 0, 5) == 0.0
      assert Covariance6.get(cov, 5, 0) == 0.0
    end
  end

  describe "diagonal/1" do
    test "from a list of six numbers" do
      cov = Covariance6.diagonal([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

      assert Covariance6.get(cov, 0, 0) == 1.0
      assert Covariance6.get(cov, 5, 5) == 6.0
      assert Covariance6.get(cov, 0, 5) == 0.0
    end

    test "from a {6} tensor" do
      cov = Covariance6.diagonal(Nx.tensor([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], type: :f64))
      assert Covariance6.get(cov, 3, 3) == 0.4
    end

    test "rejects non-six lists" do
      assert_raise ArgumentError, fn -> Covariance6.diagonal([1.0, 2.0]) end
    end
  end
end
