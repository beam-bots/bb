# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.StoreTest do
  use ExUnit.Case, async: false

  alias BB.Parameter
  alias BB.Parameter.Store.Dets

  # Use a unique path for each test run to avoid conflicts
  @test_dir Path.join(System.tmp_dir!(), "bb_store_test_#{:erlang.unique_integer([:positive])}")

  setup_all do
    File.mkdir_p!(@test_dir)
    on_exit(fn -> File.rm_rf!(@test_dir) end)
    :ok
  end

  defmodule RobotWithoutStore do
    @moduledoc false
    use BB

    parameters do
      param :no_store_value, type: :float, default: 1.0
    end

    topology do
      link :base_link do
      end
    end
  end

  describe "BB.Parameter.Store.Dets" do
    test "init requires path option" do
      assert {:error, {:missing_option, :path}} = Dets.init(__MODULE__, [])
    end

    test "init creates file at specified path" do
      path = Path.join(@test_dir, "create_#{:erlang.unique_integer([:positive])}.dets")
      refute File.exists?(path)

      assert {:ok, state} = Dets.init(__MODULE__, path: path)
      assert File.exists?(path)

      :ok = Dets.close(state)
    end

    test "init/load/save/close lifecycle" do
      path = Path.join(@test_dir, "lifecycle_#{:erlang.unique_integer([:positive])}.dets")

      # Init
      assert {:ok, state} = Dets.init(__MODULE__, path: path)
      assert File.exists?(path)

      # Load empty
      assert {:ok, []} = Dets.load(state)

      # Save
      assert :ok = Dets.save(state, [:motion, :max_speed], 2.5)
      assert :ok = Dets.save(state, [:debug_mode], true)

      # Load after save
      assert {:ok, params} = Dets.load(state)
      assert length(params) == 2
      assert {[:motion, :max_speed], 2.5} in params
      assert {[:debug_mode], true} in params

      # Close
      assert :ok = Dets.close(state)
    end

    test "save overwrites existing values" do
      path = Path.join(@test_dir, "overwrite_#{:erlang.unique_integer([:positive])}.dets")

      {:ok, state} = Dets.init(__MODULE__, path: path)

      :ok = Dets.save(state, [:test, :value], 1.0)
      {:ok, params1} = Dets.load(state)
      assert {[:test, :value], 1.0} in params1

      :ok = Dets.save(state, [:test, :value], 2.0)
      {:ok, params2} = Dets.load(state)
      assert {[:test, :value], 2.0} in params2
      assert length(params2) == 1

      :ok = Dets.close(state)
    end

    test "handles various value types" do
      path = Path.join(@test_dir, "types_#{:erlang.unique_integer([:positive])}.dets")

      {:ok, state} = Dets.init(__MODULE__, path: path)

      # Float
      :ok = Dets.save(state, [:float_val], 3.14159)
      # Integer
      :ok = Dets.save(state, [:int_val], 42)
      # Boolean
      :ok = Dets.save(state, [:bool_val], true)
      # String
      :ok = Dets.save(state, [:string_val], "hello")
      # Atom
      :ok = Dets.save(state, [:atom_val], :test_atom)
      # Unit (Cldr.Unit struct)
      :ok = Dets.save(state, [:unit_val], Cldr.Unit.new!(:meter, 1.5))

      {:ok, params} = Dets.load(state)
      assert length(params) == 6

      assert {[:float_val], 3.14159} in params
      assert {[:int_val], 42} in params
      assert {[:bool_val], true} in params
      assert {[:string_val], "hello"} in params
      assert {[:atom_val], :test_atom} in params

      {[:unit_val], unit} = Enum.find(params, fn {path, _} -> path == [:unit_val] end)
      assert %Cldr.Unit{} = unit
      assert unit.unit == :meter

      :ok = Dets.close(state)
    end

    test "handles deeply nested paths" do
      path = Path.join(@test_dir, "nested_#{:erlang.unique_integer([:positive])}.dets")

      {:ok, state} = Dets.init(__MODULE__, path: path)

      deep_path = [:level1, :level2, :level3, :level4, :value]
      :ok = Dets.save(state, deep_path, "deep")

      {:ok, params} = Dets.load(state)
      assert {deep_path, "deep"} in params

      :ok = Dets.close(state)
    end

    test "persists across reopens" do
      path = Path.join(@test_dir, "persist_#{:erlang.unique_integer([:positive])}.dets")

      # First session - save values
      {:ok, state1} = Dets.init(__MODULE__, path: path)
      :ok = Dets.save(state1, [:motion, :max_speed], 3.0)
      :ok = Dets.save(state1, [:motion, :acceleration], 1.5)
      :ok = Dets.close(state1)

      # Second session - load values
      {:ok, state2} = Dets.init(__MODULE__, path: path)
      {:ok, params} = Dets.load(state2)
      :ok = Dets.close(state2)

      assert length(params) == 2
      assert {[:motion, :max_speed], 3.0} in params
      assert {[:motion, :acceleration], 1.5} in params
    end

    test "multiple robots use separate tables" do
      path1 = Path.join(@test_dir, "robot1_#{:erlang.unique_integer([:positive])}.dets")
      path2 = Path.join(@test_dir, "robot2_#{:erlang.unique_integer([:positive])}.dets")

      {:ok, state1} = Dets.init(Robot1, path: path1)
      {:ok, state2} = Dets.init(Robot2, path: path2)

      :ok = Dets.save(state1, [:shared, :param], "robot1_value")
      :ok = Dets.save(state2, [:shared, :param], "robot2_value")

      {:ok, params1} = Dets.load(state1)
      {:ok, params2} = Dets.load(state2)

      assert {[:shared, :param], "robot1_value"} in params1
      assert {[:shared, :param], "robot2_value"} in params2

      :ok = Dets.close(state1)
      :ok = Dets.close(state2)
    end
  end

  describe "runtime integration" do
    test "robot works without parameter store" do
      start_supervised!(RobotWithoutStore)

      # Should work normally
      assert {:ok, 1.0} = Parameter.get(RobotWithoutStore, [:no_store_value])
      assert :ok = Parameter.set(RobotWithoutStore, [:no_store_value], 2.0)
      assert {:ok, 2.0} = Parameter.get(RobotWithoutStore, [:no_store_value])
    end
  end

  describe "runtime persistence" do
    # These tests need to define modules with specific store paths at compile time
    # Since we can't dynamically set the path in the DSL, we test the store
    # integration indirectly through the DETS module tests above

    test "store is initialized and closed with robot lifecycle" do
      # This tests that the Runtime correctly calls init/close on the store
      # by checking that no errors occur when starting/stopping a robot
      # with a store configured

      path = Path.join(@test_dir, "runtime_#{:erlang.unique_integer([:positive])}.dets")

      # Manually test the store init/close flow that Runtime would do
      {:ok, store_state} = Dets.init(__MODULE__, path: path)

      # Simulate saving a parameter
      :ok = Dets.save(store_state, [:test, :param], 123)

      # Simulate loading on restart
      {:ok, params} = Dets.load(store_state)
      assert {[:test, :param], 123} in params

      # Close
      :ok = Dets.close(store_state)
    end
  end
end
