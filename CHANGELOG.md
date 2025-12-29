<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](Https://conventionalcommits.org) for commit guidelines.

<!-- changelog -->

## [v0.10.0](https://github.com/beam-bots/bb/compare/v0.9.0...v0.10.0) (2025-12-29)




### Features:

* add simulation mode for running robots without hardware (#21) by James Harton

* add Vec3 and Quaternion modules with Transform integration (#22) by James Harton

## [v0.9.0](https://github.com/beam-bots/bb/compare/v0.8.0...v0.9.0) (2025-12-26)




### Features:

* add diagnostic and performance telemetry by James Harton

* add structured error handling with `splode` by James Harton

### Improvements:

* add `@type t` to `BB.Error.Invalid.JointConfig` by James Harton

* make `BB.Safety.disarm/2` timeout configurable by James Harton

## [v0.8.0](https://github.com/beam-bots/bb/compare/v0.7.0...v0.8.0) (2025-12-24)




### Features:

* add param() references and wrapper GenServer pattern (#19) by James Harton

* parameters: allow setting params via `start_link` options by James Harton

* dsl: add `param()` references for topology fields by James Harton

* dsl: add `param()` references in actuator/sensor/controller options by James Harton

### Improvements:

* fix arm/disarm actions by James Harton

## [v0.7.0](https://github.com/beam-bots/bb/compare/v0.6.0...v0.7.0) (2025-12-20)




### Features:

* safety: add hardware error reporting with auto-disarm (#16) by James Harton

## [v0.6.0](https://github.com/beam-bots/bb/compare/v0.5.0...v0.6.0) (2025-12-20)




### Features:

* add GenServer behaviours with options_schema callbacks (#15) by James Harton

## [v0.5.0](https://github.com/beam-bots/bb/compare/v0.4.0...v0.5.0) (2025-12-18)




### Features:

* add motion integration for IK solving and actuator commands (#14) by James Harton

### Bug Fixes:

* move argument type docs to entity docs field by James Harton

### Improvements:

* make alpha channel of color optional by James Harton

* motion: add joint state publishing and flexible target formats by James Harton

## [v0.4.0](https://github.com/beam-bots/bb/compare/v0.3.0...v0.4.0) (2025-12-13)




### Features:

* add BB.Safety system for centralised arm/disarm control (#10) by James Harton

* add BB.Safety system for centralised arm/disarm control by James Harton

### Improvements:

* concurrent disarm callbacks and :disarming state by James Harton

* refactor terminate callback and add safety docs to CLAUDE.md by James Harton

## [v0.3.0](https://github.com/beam-bots/bb/compare/v0.2.1...v0.3.0) (2025-12-13)




### Features:

* add standard actuator command interface (#9) by James Harton

* add `BB.Message.Actuator.EndMotion` (#8) by James Harton

* add `BB.Sensor.OpenLoopPositionEstimator` (#7) by James Harton

## [v0.2.1](https://github.com/beam-bots/bb/compare/v0.2.0...v0.2.1) (2025-12-09)




## [v0.2.0](https://github.com/beam-bots/bb/compare/v0.1.0...v0.2.0) (2025-12-06)
### Breaking Changes:

* rename project from Kinetix to Beam Bots by James Harton

* change axis DSL from translational to rotational units by James Harton

* move `name` option from `robot` to `settings` section by James Harton

* restructure DSL with top-level sections by James Harton

* refactor command execution to task-based model by James Harton

* add robot_sensors and controllers DSL sections by James Harton



### Features:

* add parameter system for runtime-adjustable configuration by James Harton

* add Igniter install tasks for project scaffolding by James Harton

* add URDF export mix task (#8) by James Harton

* add robot state machine for command control by James Harton

* add process communication functions and registry partitioning by James Harton

* add hierarchical pubsub system for robot component messages by James Harton

* add optimised robot representation with forward kinematics by James Harton

* add topology-based supervision tree for fault isolation by James Harton

* add actuator entity and sensors to joints by James Harton

* add sensor DSL entity for defining robot sensors by James Harton

* add foundational message system for robot component communication (#5) by James Harton

* Add basic robot definition DSL (#4) by James Harton

## [v0.1.0](https://github.com/beam-bots/bb/compare/v0.1.0...v0.1.0) (2025-11-27)



