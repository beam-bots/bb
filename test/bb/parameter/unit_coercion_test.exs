# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Parameter.UnitCoercionTest do
  @moduledoc """
  A unit-typed parameter accepts any value compatible with its declared unit.
  Every path which writes one has to converge on the declared unit, or the
  parameter reports back whichever unit it happened to be written in.
  """
  use ExUnit.Case, async: false

  import BB.Unit

  alias BB.Parameter
  alias BB.Unit.Option, as: UnitOption
  alias Localize.Unit, as: LocalizeUnit

  defmodule RadianStore do
    @moduledoc false
    @behaviour BB.Parameter.Store

    @impl BB.Parameter.Store
    def init(_robot_module, _opts), do: {:ok, %{}}

    @impl BB.Parameter.Store
    def load(_state), do: {:ok, [{[:motion, :trim], LocalizeUnit.new!(0.5, "radian")}]}

    @impl BB.Parameter.Store
    def save(_state, _path, _value), do: :ok

    @impl BB.Parameter.Store
    def close(_state), do: :ok
  end

  defmodule RadianDefaultController do
    @moduledoc false
    @behaviour BB.Parameter

    @impl BB.Parameter
    def param_schema do
      Spark.Options.new!(
        tilt: [
          type: UnitOption.unit_type(compatible: :degree),
          default: LocalizeUnit.new!(0.25, "radian")
        ]
      )
    end
  end

  defmodule Robot do
    @moduledoc false
    use BB

    parameters do
      group :motion do
        param(:trim,
          type: {:unit, :degree},
          default: ~u(0 degree),
          min: ~u(-30 degree),
          max: ~u(30 degree)
        )

        param(:slew, type: {:unit, :degree_per_second}, default: ~u(45 degree_per_second))
      end
    end

    topology do
      link :base_link do
      end
    end
  end

  defmodule RadianDefaultRobot do
    @moduledoc false
    use BB

    parameters do
      param(:reach, type: {:unit, :meter}, default: ~u(50 centimeter))
    end

    topology do
      link :base_link do
      end
    end
  end

  defmodule PersistedRobot do
    @moduledoc false
    use BB

    settings do
      parameter_store(BB.Parameter.UnitCoercionTest.RadianStore)
    end

    parameters do
      group :motion do
        param(:trim, type: {:unit, :degree}, default: ~u(0 degree))
      end
    end

    topology do
      link :base_link do
      end
    end
  end

  describe "set/3" do
    setup do
      start_supervised!(Robot)
      :ok
    end

    test "converts a compatible unit into the declared one" do
      assert :ok = Parameter.set(Robot, [:motion, :trim], ~u(0.26 radian))

      assert {:ok, %LocalizeUnit{name: "degree", value: degrees}} =
               Parameter.get(Robot, [:motion, :trim])

      assert_in_delta degrees, 14.8969, 0.0001
    end

    test "leaves a value already in the declared unit alone" do
      assert :ok = Parameter.set(Robot, [:motion, :trim], ~u(12.5 degree))
      assert Parameter.get(Robot, [:motion, :trim]) == {:ok, ~u(12.5 degree)}
    end

    test "converts a compound unit" do
      assert :ok = Parameter.set(Robot, [:motion, :slew], ~u(1 radian_per_second))

      assert {:ok, %LocalizeUnit{name: "degree-per-second", value: rate}} =
               Parameter.get(Robot, [:motion, :slew])

      assert_in_delta rate, 57.2957, 0.0001
    end

    test "still enforces bounds declared in another unit" do
      assert {:error, _reason} = Parameter.set(Robot, [:motion, :trim], ~u(1 radian))
      assert Parameter.get(Robot, [:motion, :trim]) == {:ok, ~u(0 degree)}
    end

    test "publishes the change in the declared unit" do
      BB.subscribe(Robot, [:param, :motion, :trim])

      :ok = Parameter.set(Robot, [:motion, :trim], ~u(0.1 radian))

      assert_receive {:bb, [:param, :motion, :trim], %BB.Message{payload: payload}}
      assert payload.new_value.name == "degree"
    end
  end

  describe "set_many/2" do
    setup do
      start_supervised!(Robot)
      :ok
    end

    test "converts every parameter it writes" do
      assert :ok =
               Parameter.set_many(Robot, [
                 {[:motion, :trim], ~u(0.1 radian)},
                 {[:motion, :slew], ~u(1 radian_per_second)}
               ])

      assert {:ok, %LocalizeUnit{name: "degree"}} = Parameter.get(Robot, [:motion, :trim])

      assert {:ok, %LocalizeUnit{name: "degree-per-second"}} =
               Parameter.get(Robot, [:motion, :slew])
    end

    test "writes nothing when one parameter is out of bounds" do
      assert {:error, _errors} =
               Parameter.set_many(Robot, [
                 {[:motion, :trim], ~u(1 radian)},
                 {[:motion, :slew], ~u(1 radian_per_second)}
               ])

      assert Parameter.get(Robot, [:motion, :trim]) == {:ok, ~u(0 degree)}
      assert Parameter.get(Robot, [:motion, :slew]) == {:ok, ~u(45 degree_per_second)}
    end
  end

  describe "startup values" do
    test "a DSL default declared in another unit is converted" do
      start_supervised!(RadianDefaultRobot)

      assert Parameter.get(RadianDefaultRobot, [:reach]) == {:ok, ~u(0.5 meter)}
    end

    test "a params: override is converted" do
      start_supervised!({Robot, params: [motion: [trim: ~u(0.1 radian)]]})

      assert {:ok, %LocalizeUnit{name: "degree", value: degrees}} =
               Parameter.get(Robot, [:motion, :trim])

      assert_in_delta degrees, 5.7295, 0.0001
    end

    test "a component's schema default is converted when it registers" do
      start_supervised!(Robot)

      :ok = Parameter.register(Robot, [:controller, :tilt_ctl], RadianDefaultController)

      assert {:ok, %LocalizeUnit{name: "degree", value: degrees}} =
               Parameter.get(Robot, [:controller, :tilt_ctl, :tilt])

      assert_in_delta degrees, 14.3239, 0.0001
    end

    test "a persisted value is converted when it is replayed" do
      start_supervised!(PersistedRobot)

      assert {:ok, %LocalizeUnit{name: "degree", value: degrees}} =
               Parameter.get(PersistedRobot, [:motion, :trim])

      assert_in_delta degrees, 28.6478, 0.0001
    end
  end
end
