# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

Application.ensure_all_started(:mimic)

:logger.add_primary_filter(:test_filter, {&TestLogFilter.log/2, []})

ExUnit.start(capture_log: true)

Mimic.copy(BB.PubSub)
