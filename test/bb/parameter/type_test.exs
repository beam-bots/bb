# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.TypeTest do
  use ExUnit.Case, async: true
  import BB.Unit

  alias BB.Parameter.Type

  doctest BB.Parameter.Type

  describe "option_type/3" do
    test "leaves an unbounded simple type alone" do
      assert {:ok, :boolean} = Type.option_type(:boolean, nil, nil)
    end

    test "converts an unbounded unit type to a unit constraint" do
      assert {:ok, {:custom, BB.Unit.Option, :validate, [[compatible: :meter]]}} =
               Type.option_type({:unit, :meter}, nil, nil)
    end

    test "keeps only the bound that was given" do
      assert {:ok, {:custom, Type, :validate_bounds, [[type: :float, min: nil, max: 1.0]]}} =
               Type.option_type(:float, nil, 1.0)
    end

    test "rejects a bound which is not a number" do
      assert {:error, message} = Type.option_type(:float, :nope, nil)
      assert message =~ "`min` must be a number"
    end

    test "rejects bounds on a non-numeric type" do
      for type <- [:boolean, :string, :atom] do
        assert {:error, message} = Type.option_type(type, 0, 1)
        assert message =~ "only supported for numeric parameter types"
      end
    end

    test "rejects min greater than max" do
      assert {:error, message} = Type.option_type(:integer, 10, 1)
      assert message =~ "`min` must not be greater than `max`"
    end

    test "rejects unit bounds greater than max" do
      assert {:error, message} =
               Type.option_type({:unit, :meter}, ~u(3 meter), ~u(2 meter))

      assert message =~ "`min` must not be greater than `max`"
    end

    test "accepts unit bounds in any compatible unit" do
      assert {:ok, {:custom, BB.Unit.Option, :validate, [options]}} =
               Type.option_type({:unit, :meter}, ~u(0 meter), ~u(150 centimeter))

      assert options[:compatible] == :meter
      assert Localize.Unit.compare(options[:max], ~u(1.5 meter)) == :eq
    end
  end

  describe "validate_bounds/2" do
    test "accepts a value on either bound" do
      bounds = [type: :integer, min: 0, max: 127]

      assert {:ok, 0} = Type.validate_bounds(0, bounds)
      assert {:ok, 127} = Type.validate_bounds(127, bounds)
    end

    test "rejects a float for an integer parameter" do
      assert {:error, "expected integer, got: 1.5"} =
               Type.validate_bounds(1.5, type: :integer, min: 0, max: 127)
    end

    test "rejects an integer for a float parameter" do
      assert {:error, "expected float, got: 1"} =
               Type.validate_bounds(1, type: :float, min: 0.0, max: 2.0)
    end
  end

  describe "describe/1" do
    test "reports hand-written component schema types unchanged" do
      assert {{:in, [:a, :b]}, nil, nil} = Type.describe({:in, [:a, :b]})
      assert {nil, nil, nil} = Type.describe(nil)
    end
  end
end
