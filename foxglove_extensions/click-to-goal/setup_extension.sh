#!/usr/bin/env bash
set -e

echo "=== Click-to-goal Foxglove Extension Setup ==="

# 1. Check that we're in the correct folder
if [ ! -f "package.json" ]; then
  echo "ERROR: package.json not found."
  echo "Please run this script from the extension root directory, e.g.:"
  echo "  foxglove_extensions/click-to-goal"
  exit 1
fi

# 2. Check for node
if ! command -v node >/dev/null 2>&1; then
  echo "ERROR: Node.js is not installed or not in PATH."
  echo "Please install Node.js (recommended v18.x) and try again."
  exit 1
fi

# 3. Check for npm
if ! command -v npm >/dev/null 2>&1; then
  echo "ERROR: npm is not installed or not in PATH."
  echo "Please install npm and try again."
  exit 1
fi

echo "Using Node: $(node -v)"
echo "Using npm:  $(npm -v)"
echo

# 4. Install dependencies
echo "=== Installing npm dependencies (npm install) ==="
npm install

# 5. Build the extension
echo
echo "=== Building extension (npm run build) ==="
npm run build

# 6. Install into Foxglove Desktop
echo
echo "=== Installing into Foxglove Desktop (npm run local-install) ==="
npm run local-install

echo
echo "✅ Done!"
echo "- Restart Foxglove Desktop."
echo "- Open Settings → Extensions and verify that 'Click-to-goal' is listed."
echo "- Then add the Click-to-goal panel in a layout"