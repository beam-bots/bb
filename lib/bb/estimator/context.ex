# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Estimator.Context do
  @moduledoc """
  Framework-provided init context delivered to `c:BB.Estimator.init/1` via
  the `:estimator_context` option.

  Carries the topology-derived information an estimator needs to interpret
  its inputs and stamp its outputs correctly: the robot module, the
  estimator's full path, the target frame for its outputs, and the static
  frame transforms from each input's source frame to the target frame.

  ## Fields

  - `:robot` - The robot module that owns this estimator.
  - `:path` - The estimator's full path (e.g. `[:sensor, :base_link, :imu,
    :orientation]` for sensor-nested or `[:estimator, :base_link, :pose]`
    for link-nested).
  - `:target_frame` - The frame atom of the estimator's outputs. For
    sensor-nested estimators this is the parent sensor's frame. For
    link-nested estimators this is the parent link's name.
  - `:transforms` - A map from input declaration name to a
    `BB.Math.Transform.t()` describing the static transform from that
    input's source frame into the target frame. For inputs already in the
    target frame the value is `BB.Math.Transform.identity()`. Empty for
    sensor-nested estimators (no transform needed - the input is in the
    same frame as the output).
  """

  alias BB.Math.Transform

  defstruct [:robot, :path, :target_frame, transforms: %{}]

  @type t :: %__MODULE__{
          robot: module(),
          path: [atom()],
          target_frame: atom(),
          transforms: %{atom() => Transform.t()}
        }
end
