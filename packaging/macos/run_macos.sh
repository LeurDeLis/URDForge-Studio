#!/bin/bash
# ==============================================================================
# URDForge Studio - macOS One-Click Launcher
# ==============================================================================
set -e

# Resolve repository root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "=========================================================="
echo "          URDForge Studio - Starting on macOS             "
echo "=========================================================="

export PYTHONPATH="$PROJECT_ROOT/src:$PYTHONPATH"

# macOS Hardware & Metal / WebEngine Acceleration Flags
export QTWEBENGINE_CHROMIUM_FLAGS="--enable-gpu-rasterization --ignore-gpu-blocklist --enable-zero-copy"

# Detect Python environment
if [ -n "$CONDA_PREFIX" ] && [ -f "$CONDA_PREFIX/bin/python" ]; then
    PYTHON_EXEC="$CONDA_PREFIX/bin/python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_EXEC="python3"
else
    echo "❌ Error: Python 3 not found. Please install Python 3.10+ or Miniconda."
    exit 1
fi

echo "Using Python: $($PYTHON_EXEC --version) ($PYTHON_EXEC)"
exec "$PYTHON_EXEC" app.py "$@"
