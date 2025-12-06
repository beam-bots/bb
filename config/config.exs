# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

import Config

config :git_ops,
  mix_project: Mix.Project.get!(),
  changelog_file: "CHANGELOG.md",
  repository_url: "https://github.com/beam-bots/bb",
  manage_mix_version?: true,
  manage_readme_version: "README.md",
  version_tag_prefix: "v"
