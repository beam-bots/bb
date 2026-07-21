# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.UsageRulesTest do
  use ExUnit.Case, async: true

  @moduletag :usage_rules

  test "the main usage-rules.md exists" do
    assert File.regular?("usage-rules.md")
  end

  test "usage-rules/ ships focused sub-rules" do
    sub_rules = Path.wildcard("usage-rules/*.md")
    assert sub_rules != []

    for path <- sub_rules do
      assert File.read!(path) =~ "SPDX-License-Identifier",
             "#{path} is missing its SPDX header"
    end
  end

  test "usage rules are included in the hex package files" do
    files = Keyword.fetch!(Mix.Project.config()[:package], :files)
    assert "usage-rules.md" in files
    assert "usage-rules" in files
  end
end
