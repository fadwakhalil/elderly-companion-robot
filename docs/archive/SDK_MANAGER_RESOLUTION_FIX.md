# Fix SDK Manager Resolution Warning

**Quick guide to make SDK Manager GUI usable when resolution is too small.**

---

## Problem

SDK Manager requires at least 1440x900 resolution, but your VM window is smaller (e.g., 1052x411), causing the GUI to be cut off.

---

## Solutions (Try in Order)

### Solution 1: Make VM Window Larger (Easiest)

1. **In UTM:**
   - Drag the corners of the VM window to make it larger
   - Or go fullscreen: Press `Cmd+Ctrl+F` or View → Fullscreen
   - The VM should resize automatically

2. **In Ubuntu VM:**
   - Go to: Settings → Displays
   - Increase resolution to at least 1440x900 or higher
   - Apply the changes

3. **Restart SDK Manager:**
   ```bash
   sdkmanager
   ```

---

### Solution 2: Change UTM Display Settings

1. **In UTM, select your VM**
2. **Click "Edit"** (or right-click → Edit)
3. **Go to Display settings:**
   - Click **"Display"** in left sidebar
   - **Emulated Display Card:** Try "virtio-gpu-gl-pci" (GPU Supported) for better performance
   - ✅ Check **"Resize display to window size automatically"**
   - **Retina Mode:** ✅ Check if you have Retina display
4. **Click "Save"**
5. **Restart VM** and try SDK Manager again

---

### Solution 3: Increase Ubuntu Resolution

1. **In Ubuntu VM:**
   ```bash
   # Open display settings
   gnome-control-center display
   ```
   Or: Settings → Displays

2. **Change Resolution:**
   - Select a higher resolution (1440x900 or higher)
   - Click "Apply"
   - Keep the new resolution if it looks good

3. **If resolution options are limited:**
   ```bash
   # Install additional display tools
   sudo apt update
   sudo apt install -y xrandr
   
   # List available resolutions
   xrandr
   
   # Set custom resolution (example)
   xrandr --output Virtual-1 --mode 1920x1080
   ```

---

### Solution 4: Use Fullscreen Mode

1. **In UTM:**
   - Press `Cmd+Ctrl+F` to enter fullscreen
   - Or: View → Fullscreen
   - This gives maximum screen space

2. **In Ubuntu:**
   - Press `Super` (Windows key) to open Activities
   - Go to Settings → Displays
   - Set to highest available resolution

---

### Solution 5: Use CLI Instead (If GUI Still Doesn't Work)

If the GUI is still unusable, use the command-line interface:

```bash
# See available commands
sdkmanager --help

# List available JetPack versions
sdkmanager --list

# Flash Jetson via CLI
# sdkmanager --cli install --logintype devzone --product Jetson --target JETSON_ORIN_NANO_DEVKIT --version JP_6.2
```

---

## Quick Checklist

- [ ] Made VM window larger (dragged corners)
- [ ] Tried fullscreen mode (`Cmd+Ctrl+F`)
- [ ] Increased Ubuntu display resolution (Settings → Displays)
- [ ] Changed UTM display card to "virtio-gpu-gl-pci"
- [ ] Enabled "Resize display to window size automatically"
- [ ] Restarted SDK Manager

---

## Recommended Settings

**UTM Display Settings:**
- Emulated Display Card: `virtio-gpu-gl-pci` (GPU Supported)
- Auto Resolution: ✅ Enabled
- Retina Mode: ✅ Enabled (if Retina display)

**Ubuntu Display Settings:**
- Resolution: 1920x1080 or 1440x900 minimum
- Scale: 100% (or adjust as needed)

---

**After making changes, restart SDK Manager and the resolution warning should be gone!**

