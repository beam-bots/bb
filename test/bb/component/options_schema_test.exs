# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Component.OptionsSchemaTest do
  use ExUnit.Case, async: true

  alias BB.Component.OptionsSchema

  defmodule Fixture do
    @moduledoc false
    use BB.Sensor,
      options_schema: [
        beta: [type: :float, default: 0.1],
        accel_threshold: [type: :float, default: 0.05],
        gain: [type: :integer, required: true]
      ]

    @impl BB.Sensor
    def init(opts), do: {:ok, opts}
  end

  @framework_keys [:bb, :sensor_profile]

  describe "validate/3" do
    test "applies schema defaults for keys the caller omitted" do
      {:ok, validated} =
        OptionsSchema.validate(
          Fixture,
          [bb: :fake_bb, sensor_profile: :fake_profile, gain: 7],
          @framework_keys
        )

      assert validated[:gain] == 7
      assert validated[:beta] == 0.1
      assert validated[:accel_threshold] == 0.05
    end

    test "preserves framework-injected keys verbatim, not validated by the schema" do
      bb = %{robot: :robot_mod, path: [:base, :imu]}

      {:ok, validated} =
        OptionsSchema.validate(
          Fixture,
          [bb: bb, sensor_profile: :prof, gain: 3],
          @framework_keys
        )

      assert validated[:bb] == bb
      assert validated[:sensor_profile] == :prof
    end

    test "returns a validation error when a user key is not in the schema" do
      assert {:error, %Spark.Options.ValidationError{}} =
               OptionsSchema.validate(
                 Fixture,
                 [bb: :fake, gain: 1, surprise: :unexpected],
                 @framework_keys
               )
    end

    test "returns a validation error when a required key is missing" do
      assert {:error, %Spark.Options.ValidationError{}} =
               OptionsSchema.validate(Fixture, [bb: :fake], @framework_keys)
    end
  end

  describe "schema declaration" do
    test "passing options_schema: to `use` AND defining options_schema/0 is a compile error" do
      assert_raise CompileError, ~r/Declare the schema one way, not both/, fn ->
        defmodule DoubleDeclaration do
          @moduledoc false
          use BB.Sensor, options_schema: [foo: [type: :integer]]

          @impl BB.Sensor
          def options_schema, do: Spark.Options.new!([])

          @impl BB.Sensor
          def init(opts), do: {:ok, opts}
        end
      end
    end

    test "omitting the schema entirely yields an empty default options_schema/0" do
      defmodule NoSchema do
        @moduledoc false
        use BB.Sensor

        @impl BB.Sensor
        def init(opts), do: {:ok, opts}
      end

      assert %Spark.Options{schema: []} = NoSchema.options_schema()
    end
  end
end
