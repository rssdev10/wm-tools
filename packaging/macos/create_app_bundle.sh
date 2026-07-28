#!/bin/bash

# Script to create a macOS .app bundle for DSO3D12 GUI.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CREATE_DMG=false
APP_NAME="DSO3D12"
BUNDLE_NAME="DSO3D12.app"
DMG_NAME="DSO3D12.dmg"
IDENTIFIER="com.dso3d12.gui"
VERSION="0.1.0"
ASSETS_DIR="$SCRIPT_DIR"
ICON_SOURCE="$PROJECT_ROOT/gui/img/dso3d12_ico.png"
OUTPUT_DIR="$PROJECT_ROOT"
BINARY_PATH=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dmg)
            CREATE_DMG=true
            shift
            ;;
        --binary-path)
            BINARY_PATH="$2"
            shift 2
            ;;
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --assets-dir)
            ASSETS_DIR="$2"
            shift 2
            ;;
        --icon)
            ICON_SOURCE="$2"
            shift 2
            ;;
        --version)
            VERSION="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1
            ;;
    esac
done

if [[ -z "$BINARY_PATH" ]]; then
    BINARY_PATH="$PROJECT_ROOT/target/release/dso3d12-gui"
fi

INFO_TEMPLATE="$ASSETS_DIR/Info.plist.template"
APP_DIR="$OUTPUT_DIR/$BUNDLE_NAME"
DMG_PATH="$OUTPUT_DIR/$DMG_NAME"

if [[ ! -f "$BINARY_PATH" ]]; then
    echo "Binary not found: $BINARY_PATH" >&2
    exit 1
fi

if [[ ! -f "$INFO_TEMPLATE" ]]; then
    echo "Info.plist template not found: $INFO_TEMPLATE" >&2
    exit 1
fi

if [[ ! -f "$ICON_SOURCE" ]]; then
    echo "Icon source not found: $ICON_SOURCE" >&2
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

echo "Creating macOS app bundle: $APP_DIR"

rm -rf "$APP_DIR"
mkdir -p "$APP_DIR/Contents/MacOS"
mkdir -p "$APP_DIR/Contents/Resources"

echo "Copying binary..."
cp "$BINARY_PATH" "$APP_DIR/Contents/MacOS/dso3d12-gui"
chmod +x "$APP_DIR/Contents/MacOS/dso3d12-gui"

echo "Converting icon to PNG (512px)..."
sips -s format png "$ICON_SOURCE" --out "$APP_DIR/Contents/Resources/icon.png" --resampleWidth 512 > /dev/null 2>&1

echo "Creating icon set..."
ICONSET="$APP_DIR/Contents/Resources/icon.iconset"
mkdir -p "$ICONSET"

sips -z 16 16     "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_16x16.png" > /dev/null 2>&1
sips -z 32 32     "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_16x16@2x.png" > /dev/null 2>&1
sips -z 32 32     "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_32x32.png" > /dev/null 2>&1
sips -z 64 64     "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_32x32@2x.png" > /dev/null 2>&1
sips -z 128 128   "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_128x128.png" > /dev/null 2>&1
sips -z 256 256   "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_128x128@2x.png" > /dev/null 2>&1
sips -z 256 256   "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_256x256.png" > /dev/null 2>&1
sips -z 512 512   "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_256x256@2x.png" > /dev/null 2>&1
sips -z 512 512   "$APP_DIR/Contents/Resources/icon.png" --out "$ICONSET/icon_512x512.png" > /dev/null 2>&1
cp "$APP_DIR/Contents/Resources/icon.png" "$ICONSET/icon_512x512@2x.png"

echo "Converting to .icns format..."
iconutil -c icns "$ICONSET" -o "$APP_DIR/Contents/Resources/icon.icns"

rm -rf "$ICONSET"
rm "$APP_DIR/Contents/Resources/icon.png"

echo "Creating Info.plist..."
sed \
    -e "s|__APP_NAME__|$APP_NAME|g" \
    -e "s|__IDENTIFIER__|$IDENTIFIER|g" \
    -e "s|__VERSION__|$VERSION|g" \
    "$INFO_TEMPLATE" > "$APP_DIR/Contents/Info.plist"

echo ""
echo "App bundle created successfully: $APP_DIR"
echo ""

if [[ "$CREATE_DMG" == true ]]; then
    echo "Creating DMG disk image..."
    rm -f "$DMG_PATH"
    hdiutil create -volname "$APP_NAME" -srcfolder "$APP_DIR" -ov -format UDZO "$DMG_PATH"
    echo ""
    echo "DMG created: $DMG_PATH"
    echo ""
else
    echo "To run the app:"
    echo "  open $APP_DIR"
    echo ""
    echo "To create a DMG for distribution:"
    echo "  ./create_app_bundle.sh --dmg"
    echo ""
fi
