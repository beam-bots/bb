# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

[
  tools: [
    {:credo, "mix credo --strict"},
    {:hex_audit, "mix hex.audit"},
    {:mix_audit, false},
    {:spark_formatter, "mix spark.formatter --check"},
    {:spark_cheat_sheets, "mix spark.cheat_sheets --check"},
    {:reuse, command: ["pipx", "run", "--spec", "reuse[charset-normalizer]", "reuse", "lint", "-q"]}
  ]
]
