#!/bin/bash
# Build custom highlight.js with RIDL + ROS IDL language support for mdbook.
#
# Language modules MUST be baked into theme/highlight.js (not loaded via
# book.toml additional-js): mdBook runs highlightAll() before additional-js
# executes, so a language registered there is too late.
#
# Run before mdbook build, or as part of the build pipeline.

set -e
cd "$(dirname "$0")/.."

# Copy bundled mermaid.min.js / mermaid-init.js (listed in book.toml additional-js, gitignored).
mdbook-mermaid install .

# First build to get default highlight.js (mdBook >=0.4.0 puts it under book/)
mdbook build

# Try both hashed and non-hashed names, be tolerant across mdBook versions
HLJS=$(ls book/highlight*.js 2>/dev/null | head -1)
if [ -z "$HLJS" ]; then
  echo "Error: mdbook build did not produce any highlight*.js under book/."
  exit 1
fi

# Write to theme (mdbook uses theme/highlight.js when present)
mkdir -p theme
cat "$HLJS" theme/ridl-register.js rosidl-register.js > theme/highlight.js

echo "Created theme/highlight.js with RIDL + ROS IDL support"

# Rebuild with custom highlight
mdbook build
