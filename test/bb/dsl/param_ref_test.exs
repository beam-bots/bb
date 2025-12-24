# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Dsl.ParamRefTest do
  use ExUnit.Case, async: true

  alias BB.Dsl.ParamRef
  alias BB.Unit.Option

  describe "param/1" do
    test "creates a ParamRef struct with the given path" do
      ref = ParamRef.param([:motion, :max_speed])

      assert %ParamRef{path: [:motion, :max_speed]} = ref
    end

    test "expected_unit_type is nil by default" do
      ref = ParamRef.param([:some, :param])

      assert ref.expected_unit_type == nil
    end
  end

  describe "unit option validation" do
    test "accepts ParamRef and sets expected_unit_type" do
      ref = ParamRef.param([:motion, :max_speed])

      {:ok, validated} = Option.validate(ref, compatible: :meter)

      assert validated.expected_unit_type == :meter
      assert validated.path == [:motion, :max_speed]
    end

    test "preserves ParamRef path through validation" do
      ref = ParamRef.param([:limits, :shoulder, :effort])

      {:ok, validated} = Option.validate(ref, compatible: :newton_meter)

      assert validated.path == [:limits, :shoulder, :effort]
    end

    test "accepts ParamRef without compatible option" do
      ref = ParamRef.param([:some, :param])

      {:ok, validated} = Option.validate(ref, [])

      assert validated.expected_unit_type == nil
    end
  end
end
