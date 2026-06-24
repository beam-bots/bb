# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.BridgeTest do
  use ExUnit.Case, async: true

  alias BB.Error.Invalid.Bridge.InvalidParamId
  alias BB.Error.Invalid.Bridge.ReadOnly
  alias BB.Error.Invalid.Bridge.TorqueMustBeDisabled
  alias BB.Error.Invalid.Bridge.UnknownParam

  describe "InvalidParamId" do
    test "carries the offending param_id and is an error" do
      err = InvalidParamId.exception(param_id: "nope")
      assert err.param_id == "nope"
      assert BB.Error.severity(err) == :error
    end

    test "message describes the expected format" do
      message = InvalidParamId.message(InvalidParamId.exception(param_id: "nope"))
      assert message =~ "\"nope\""
      assert message =~ "servo_id:param_name"
    end
  end

  describe "ReadOnly" do
    test "carries the param_name and is an error" do
      err = ReadOnly.exception(param_name: :firmware_version)
      assert err.param_name == :firmware_version
      assert BB.Error.severity(err) == :error
    end

    test "message names the parameter" do
      message = ReadOnly.message(ReadOnly.exception(param_name: :firmware_version))
      assert message =~ ":firmware_version"
    end
  end

  describe "TorqueMustBeDisabled" do
    test "carries param_name and servo_id and is an error" do
      err = TorqueMustBeDisabled.exception(param_name: :max_torque, servo_id: 3)
      assert err.param_name == :max_torque
      assert err.servo_id == 3
      assert BB.Error.severity(err) == :error
    end

    test "message includes the servo when present" do
      message =
        TorqueMustBeDisabled.message(
          TorqueMustBeDisabled.exception(param_name: :max_torque, servo_id: 3)
        )

      assert message =~ ":max_torque"
      assert message =~ "servo 3"
    end

    test "message omits the servo when nil" do
      message =
        TorqueMustBeDisabled.message(
          TorqueMustBeDisabled.exception(param_name: :max_torque, servo_id: nil)
        )

      assert message =~ ":max_torque"
      refute message =~ "servo"
    end
  end

  describe "UnknownParam" do
    test "carries param_name and control_table and is an error" do
      err = UnknownParam.exception(param_name: :bogus, control_table: SomeTable)
      assert err.param_name == :bogus
      assert err.control_table == SomeTable
      assert BB.Error.severity(err) == :error
    end

    test "message includes the control table when present" do
      message =
        UnknownParam.message(UnknownParam.exception(param_name: :bogus, control_table: SomeTable))

      assert message =~ ":bogus"
      assert message =~ "SomeTable"
    end

    test "message omits the control table when nil" do
      message =
        UnknownParam.message(UnknownParam.exception(param_name: :bogus, control_table: nil))

      assert message =~ ":bogus"
      refute message =~ "not found"
    end
  end
end
