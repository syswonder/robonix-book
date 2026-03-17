#!/bin/bash
# Build custom highlight.js with RIDL language support for mdbook.
# Run before mdbook build, or as part of the build pipeline.

set -e
cd "$(dirname "$0")/.."

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
cat "$HLJS" theme/ridl-register.js > theme/highlight.js

echo "Created theme/highlight.js with RIDL support"

# Rebuild to use custom highlight
mdbook build
