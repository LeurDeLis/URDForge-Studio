#!/bin/bash
# ==============================================================================
# URDForge Studio - macOS Dependency Setup & Installation Script
# ==============================================================================
set -e

echo "=========================================================="
echo "    URDForge Studio - Setting up macOS Environment        "
echo "=========================================================="

# Check if conda / mamba is available (Recommended for OpenCASCADE / OCP)
if command -v conda >/dev/null 2>&1; then
    echo "🟢 Conda detected. Creating/updating 'urdforge' conda environment..."
    conda create -y -n urdforge python=3.11
    conda run -n urdforge conda install -y -c conda-forge ocp
    conda run -n urdforge pip install fastapi uvicorn pydantic numpy scipy PySide6 open3d
    echo "✅ Environment configured! Run with:"
    echo "   conda activate urdforge"
    echo "   ./packaging/macos/run_macos.sh"
    exit 0
fi

# Fallback: Homebrew + pip
if command -v brew >/dev/null 2>&1; then
    echo "🟢 Homebrew detected. Installing prerequisites..."
    brew install python@3.11
    python3.11 -m venv ~/.urdforge-env
    source ~/.urdforge-env/bin/activate
    pip install --upgrade pip
    pip install fastapi uvicorn pydantic numpy scipy PySide6
    echo "⚠️ Note: For full OpenCASCADE STEP parsing, install Miniconda/Miniforge and 'conda install -c conda-forge ocp'."
    echo "✅ Setup complete. Launch via:"
    echo "   source ~/.urdforge-env/bin/activate && ./packaging/macos/run_macos.sh"
    exit 0
fi

echo "❌ Please install Homebrew (https://brew.sh) or Miniconda (https://docs.conda.io) first."
exit 1
