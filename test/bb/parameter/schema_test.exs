# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.SchemaTest do
  use ExUnit.Case, async: true

  alias BB.Parameter.Schema

  describe "build_nested_schema/1" do
    test "handles single-element paths" do
      flat = [
        {[:debug_mode], [type: :boolean, default: false]},
        {[:log_level], [type: :atom, default: :info]}
      ]

      schema = Schema.build_nested_schema(flat)

      assert schema == [
               debug_mode: [type: :boolean],
               log_level: [type: :atom]
             ]
    end

    test "handles nested paths" do
      flat = [
        {[:motion, :max_speed], [type: :float, default: 1.0]},
        {[:motion, :acceleration], [type: :float, default: 0.5]}
      ]

      schema = Schema.build_nested_schema(flat)

      assert schema == [
               motion: [
                 type: :keyword_list,
                 keys: [
                   acceleration: [type: :float],
                   max_speed: [type: :float]
                 ]
               ]
             ]
    end

    test "handles mixed depth paths" do
      flat = [
        {[:debug_mode], [type: :boolean, default: false]},
        {[:motion, :max_speed], [type: :float, default: 1.0]},
        {[:motion, :acceleration], [type: :float, default: 0.5]}
      ]

      schema = Schema.build_nested_schema(flat)

      assert schema == [
               debug_mode: [type: :boolean],
               motion: [
                 type: :keyword_list,
                 keys: [
                   acceleration: [type: :float],
                   max_speed: [type: :float]
                 ]
               ]
             ]
    end

    test "handles deeply nested paths" do
      flat = [
        {[:link, :arm, :joint, :elbow, :max_torque], [type: :float, default: 10.0]},
        {[:link, :arm, :joint, :elbow, :max_speed], [type: :float, default: 1.0]}
      ]

      schema = Schema.build_nested_schema(flat)

      assert schema == [
               link: [
                 type: :keyword_list,
                 keys: [
                   arm: [
                     type: :keyword_list,
                     keys: [
                       joint: [
                         type: :keyword_list,
                         keys: [
                           elbow: [
                             type: :keyword_list,
                             keys: [
                               max_speed: [type: :float],
                               max_torque: [type: :float]
                             ]
                           ]
                         ]
                       ]
                     ]
                   ]
                 ]
               ]
             ]
    end

    test "removes :default from opts" do
      flat = [{[:max_speed], [type: :float, default: 1.0, doc: "Maximum speed"]}]

      schema = Schema.build_nested_schema(flat)

      assert schema == [max_speed: [type: :float, doc: "Maximum speed"]]
    end

    test "removes :required from opts" do
      flat = [{[:max_speed], [type: :float, required: true]}]

      schema = Schema.build_nested_schema(flat)

      assert schema == [max_speed: [type: :float]]
    end

    test "handles empty input" do
      assert Schema.build_nested_schema([]) == []
    end

    test "validates correctly with Spark.Options" do
      flat = [
        {[:debug_mode], [type: :boolean, default: false]},
        {[:motion, :max_speed], [type: :float, default: 1.0]}
      ]

      schema = Schema.build_nested_schema(flat)

      assert {:ok, validated} =
               Spark.Options.validate([debug_mode: true, motion: [max_speed: 2.0]], schema)

      assert validated[:debug_mode] == true
      assert validated[:motion][:max_speed] == 2.0
    end

    test "Spark.Options rejects unknown keys" do
      flat = [{[:motion, :max_speed], [type: :float, default: 1.0]}]
      schema = Schema.build_nested_schema(flat)

      assert {:error, %Spark.Options.ValidationError{}} =
               Spark.Options.validate([motion: [unknown_param: 42]], schema)
    end

    test "Spark.Options rejects type mismatches" do
      flat = [{[:max_speed], [type: :float, default: 1.0]}]
      schema = Schema.build_nested_schema(flat)

      assert {:error, %Spark.Options.ValidationError{}} =
               Spark.Options.validate([max_speed: "not a float"], schema)
    end
  end

  describe "flatten_params/1" do
    test "flattens single-level params" do
      params = [debug_mode: true, log_level: :debug]

      flat = Schema.flatten_params(params)

      assert Enum.sort(flat) == [
               {[:debug_mode], true},
               {[:log_level], :debug}
             ]
    end

    test "flattens nested params" do
      params = [motion: [max_speed: 2.0, acceleration: 1.5]]

      flat = Schema.flatten_params(params)

      assert Enum.sort(flat) == [
               {[:motion, :acceleration], 1.5},
               {[:motion, :max_speed], 2.0}
             ]
    end

    test "flattens deeply nested params" do
      params = [link: [arm: [joint: [elbow: [max_torque: 15.0]]]]]

      flat = Schema.flatten_params(params)

      assert flat == [{[:link, :arm, :joint, :elbow, :max_torque], 15.0}]
    end

    test "handles mixed depth params" do
      params = [debug_mode: true, motion: [max_speed: 2.0]]

      flat = Schema.flatten_params(params)

      assert Enum.sort(flat) == [
               {[:debug_mode], true},
               {[:motion, :max_speed], 2.0}
             ]
    end

    test "handles empty list values as leaf values" do
      params = [empty_list: []]

      flat = Schema.flatten_params(params)

      assert flat == [{[:empty_list], []}]
    end

    test "handles empty input" do
      assert Schema.flatten_params([]) == []
    end
  end
end
