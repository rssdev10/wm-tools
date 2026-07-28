DSO3D12 GUI — macOS Unsigned App Instructions

This package contains an unsigned macOS application bundle (.app).
The application has NOT been signed with an Apple Developer Certificate.

=== Installing the Application ===

1. Extract DSO3D12.app from the zip archive (or mount the DMG)
2. Drag DSO3D12.app to your Applications folder

=== Running the Application for the First Time ===

macOS will warn about the unsigned application. To allow it to run:

METHOD 1: Right-click in Finder (Recommended)
  1. Open Finder and go to Applications
  2. Right-click on DSO3D12.app
  3. Click "Open" (not just double-click)
  4. Click "Open" in the confirmation dialog

METHOD 2: Command Line (Terminal)
  1. Open Terminal
  2. Run: xattr -d com.apple.quarantine /Applications/DSO3D12.app
  3. Then double-click the app normally

METHOD 3: System Settings
  1. Open System Settings → Privacy & Security
  2. Scroll down to look for "DSO3D12.app was blocked..."
  3. Click "Open Anyway"
  4. Double-click the app

=== Running from Terminal ===

You can also run the application from the terminal:
  /Applications/DSO3D12.app/Contents/MacOS/dso3d12-gui

=== Troubleshooting ===

If the app won't open:
- Ensure you've allowed it through System Settings
- Try the xattr command from METHOD 2 above
- Restart your Mac

If the serial port is not detected:
- Check that the oscilloscope is connected via USB
- Verify the USB-serial driver is installed (CH340/CP210x)

For more information, see the main README.md file.
