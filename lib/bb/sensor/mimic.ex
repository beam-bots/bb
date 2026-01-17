# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.Mimic do
  @moduledoc """
  A sensor that derives joint state from another joint.

  Subscribes to sensor messages from a source joint and re-publishes
  transformed messages for the mimic joint. This is useful for modelling
  parallel jaw grippers and other mechanically-linked joint pairs.

  ## Options

    * `:source` - (required) The name of the source joint to follow
    * `:multiplier` - (optional, default 1.0) Scale factor applied to position values
    * `:offset` - (optional, default 0.0) Constant offset added after scaling
    * `:message_types` - (optional, default [JointState]) List of message types to forward

  For JointState messages: `mimic_position = source_position * multiplier + offset`

  ## Example

      joint :right_finger do
        type(:prismatic)
        sensor(:mimic, {BB.Sensor.Mimic,
          source: :left_finger,
          multiplier: 1.0,
          message_types: [JointState]
        })
      end

  ## URDF Mimic Joints

  This sensor implements the equivalent of URDF mimic joints:

      <joint name="right_finger_joint" type="prismatic">
        <mimic joint="left_finger_joint" multiplier="1" offset="0"/>
      </joint>

  Forward kinematics and visualisation automatically work since they
  consume JointState messages published by this sensor.
  """

  use BB.Sensor,
    options_schema: [
      source: [type: :atom, required: true, doc: "Name of the source joint to follow"],
      multiplier: [type: :float, default: 1.0, doc: "Scale factor for position values"],
      offset: [type: :float, default: 0.0, doc: "Constant offset added after scaling"],
      message_types: [
        type: {:list, :atom},
        default: [BB.Message.Sensor.JointState],
        doc: "Message types to forward"
      ]
    ]

  alias BB.Message
  alias BB.Message.Sensor.JointState

  @impl BB.Sensor
  def init(opts) do
    {:ok, state} = build_state(opts)

    BB.subscribe(state.bb.robot, [:sensor | state.source_path],
      message_types: state.message_types
    )

    {:ok, state}
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [sensor_name, joint_name | rest] = Enum.reverse(opts.bb.path)

    source_path = build_source_path(rest, opts.source)

    state = %{
      bb: opts.bb,
      sensor_name: sensor_name,
      joint_name: joint_name,
      source: opts.source,
      source_path: source_path,
      multiplier: Map.get(opts, :multiplier, 1.0),
      offset: Map.get(opts, :offset, 0.0),
      message_types: Map.get(opts, :message_types, [JointState])
    }

    {:ok, state}
  end

  defp build_source_path(parent_path, source_joint) do
    Enum.reverse(parent_path) ++ [source_joint]
  end

  @impl BB.Sensor
  def handle_info(
        {:bb, _source_path, %Message{payload: %JointState{} = payload} = message},
        state
      ) do
    transformed_payload = transform_joint_state(payload, state)
    transformed_message = %{message | payload: transformed_payload}
    BB.publish(state.bb.robot, [:sensor | state.bb.path], transformed_message)
    {:noreply, state}
  end

  def handle_info({:bb, _source_path, %Message{} = message}, state) do
    BB.publish(state.bb.robot, [:sensor | state.bb.path], message)
    {:noreply, state}
  end

  defp transform_joint_state(%JointState{} = payload, state) do
    positions =
      Enum.map(payload.positions, fn pos ->
        pos * state.multiplier + state.offset
      end)

    names =
      Enum.map(payload.names, fn _name ->
        state.joint_name
      end)

    %{payload | positions: positions, names: names}
  end
end
