# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

Application.ensure_all_started(:mimic)

ExUnit.start()

Mimic.copy(BB.PubSub)
