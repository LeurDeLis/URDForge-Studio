#!/bin/bash
# ==============================================================================
# URDForge Studio - macOS Standalone App & DMG Packaging Tool
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

APP_NAME="URDForge-Studio"
VERSION="0.2.0"
DIST_DIR="$PROJECT_ROOT/dist"
BUILD_DIR="$PROJECT_ROOT/build"
MACOS_PKG_DIR="$PROJECT_ROOT/packaging/macos"

echo "=========================================================="
echo "    Building URDForge Studio Standalone .app for macOS    "
echo "    Version: $VERSION                                     "
echo "=========================================================="

# Check Python environment
if ! command -v python3 >/dev/null 2>&1; then
    echo "❌ Error: python3 is required to build on macOS."
    exit 1
fi

# Ensure PyInstaller is installed
python3 -m pip install --quiet pyinstaller

# Clean old dist/build
rm -rf "$DIST_DIR/$APP_NAME" "$DIST_DIR/$APP_NAME.app" "$DIST_DIR/${APP_NAME}_macOS.zip" "$DIST_DIR/${APP_NAME}_macOS.dmg"

echo "📦 Running PyInstaller on macOS..."
python3 -s -m PyInstaller \
    --clean \
    --noconfirm \
    --windowed \
    --name "$APP_NAME" \
    --add-data "src/urdforge/server/static:urdforge/server/static" \
    --add-data "src/urdforge/server/static:src/urdforge/server/static" \
    --add-data "tests/fixtures:tests/fixtures" \
    --hidden-import "OCP" \
    --hidden-import "open3d" \
    --hidden-import "numpy" \
    --hidden-import "scipy" \
    --hidden-import "PySide6.QtWebEngineWidgets" \
    --hidden-import "PySide6.QtWebEngineCore" \
    --hidden-import "PySide6.QtWidgets" \
    --hidden-import "uvicorn" \
    --hidden-import "fastapi" \
    --osx-bundle-identifier "com.urdforge.studio" \
    app.py

APP_BUNDLE="$DIST_DIR/$APP_NAME.app"

if [ -d "$APP_BUNDLE" ]; then
    echo "✅ Successfully built macOS Application Bundle: $APP_BUNDLE"

    # Copy Info.plist if custom template exists
    if [ -f "$MACOS_PKG_DIR/Info.plist" ]; then
        cp "$MACOS_PKG_DIR/Info.plist" "$APP_BUNDLE/Contents/Info.plist"
    fi

    # Create Portable ZIP distribution for macOS
    echo "📦 Creating Portable ZIP archive for macOS..."
    cd "$DIST_DIR"
    zip -q -r "${APP_NAME}-v${VERSION}-macOS-Portable.zip" "$APP_NAME.app"
    echo "✅ Created: $DIST_DIR/${APP_NAME}-v${VERSION}-macOS-Portable.zip"

    # Create DMG Disk Image if hdiutil is available (standard on macOS)
    if command -v hdiutil >/dev/null 2>&1; then
        echo "💿 Creating DMG Disk Image Installer..."
        DMG_TEMP_DIR="$PROJECT_ROOT/build/dmg_temp"
        rm -rf "$DMG_TEMP_DIR"
        mkdir -p "$DMG_TEMP_DIR"
        cp -R "$APP_BUNDLE" "$DMG_TEMP_DIR/"
        ln -s /Applications "$DMG_TEMP_DIR/Applications"

        hdiutil create -volname "URDForge Studio" -srcfolder "$DMG_TEMP_DIR" -ov -format UDZO "$DIST_DIR/${APP_NAME}-v${VERSION}-macOS.dmg"
        rm -rf "$DMG_TEMP_DIR"
        echo "🎉 Created macOS DMG Installer: $DIST_DIR/${APP_NAME}-v${VERSION}-macOS.dmg"
    fi
else
    echo "❌ Error: App bundle was not created."
    exit 1
fi
