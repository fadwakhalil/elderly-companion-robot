# UTM Setup Guide for Jetson Flashing (Apple Silicon Mac)

**UTM is a better alternative to VirtualBox for Apple Silicon Macs. It properly supports x86_64 VMs.**

---

## Why UTM Instead of VirtualBox?

- ✅ **Better x86_64 emulation** on Apple Silicon Macs
- ✅ **Free and open source**
- ✅ **Can emulate x86_64 VMs** (required for SDK Manager)
- ✅ **Works with AMD64 Ubuntu ISOs** (when using emulation mode)
- ⚠️ **Note:** Emulation will be slower than native virtualization, but it's necessary for x86_64

---

## Step 1: Install UTM

1. **Download UTM:**
   - Go to: https://mac.getutm.app/
   - Click "Download" (or get from Mac App Store)
   - Install the application

2. **Open UTM:**
   - Launch UTM from Applications
   - You may need to allow it in System Preferences → Security & Privacy

---

## Step 2: Create Ubuntu VM

1. **Click "Create a New Virtual Machine"** (or press `Cmd+N`)

2. **Choose Virtualization:**
   - ⚠️ **IMPORTANT:** Select **"Emulate"** (NOT "Virtualize")
     - "Virtualize" creates ARM64 VMs (native to Apple Silicon)
     - "Emulate" allows x86_64/AMD64 VMs (what we need for SDK Manager)
   - Click **"Continue"**

3. **Choose Operating System:**
   - Select **"Linux"**
   - Click **"Continue"**

4. **Operating System Details:**
   - **Operating System:** Linux
   - **Architecture:** Should show **"x86_64"** or **"amd64"** (not ARM64)
     - If it shows ARM64, go back and make sure you selected "Emulate" in step 2
   - **Boot ISO Image:** Click "Browse" and select your `ubuntu-22.04.5-desktop-amd64.iso`
   - Click **"Continue"**

5. **Hardware:**
   - **Memory:** Set to **8192 MB (8 GB)** or at least 4096 MB (4 GB)
   - **CPU Cores:** Set to **4** (or 2 minimum)
   - Click **"Continue"**

6. **Storage:**
   - **Disk Size:** Set to **50 GB** (or more)
   - Click **"Continue"**

7. **Summary:**
   - **Name:** `Ubuntu-Jetson-Flash`
   - Review settings
   - Click **"Save"**

---

## Step 3: Configure VM Settings

1. **Select your VM** in UTM
2. **Click "Edit"** (or right-click → Edit)

3. **USB Settings (CRITICAL for Jetson):**
   - Click **"USB"** in left sidebar
   - **USB Support:** 
     - **Start with "USB 2.0 (EHCI)"** (recommended for Jetson flashing - more stable)
     - If you get "device is not ready to flash" error, make sure you're using USB 2.0
     - USB 3.0 can be too fast/unstable for flashing in VMs
   - ✅ Check **"Share USB devices from host"** (this is the "USB Sharing" checkbox)
   - **Maximum Shared USB Devices:** 3 is fine (default)
   - This allows USB passthrough for Jetson connection
   - Click **"Save"** to apply settings
   - ⚠️ **If you get "device is not ready to flash" error, see: [FIX_DEVICE_NOT_READY_TO_FLASH.md](FIX_DEVICE_NOT_READY_TO_FLASH.md)**

4. **Display Settings (IMPORTANT for SDK Manager GUI):**
   - Click **"Display"** in left sidebar
   - **Emulated Display Card:** Try "virtio-gpu-gl-pci" or "virtio-gpu-gl-device" (GPU Supported) for better performance
   - **Auto Resolution:** ✅ Check "Resize display to window size automatically"
   - **Scaling:** 
     - Upscaling: "Linear" (better quality)
     - Downscaling: "Linear"
   - **Retina Mode:** ✅ Check this if you have a Retina display
   - Click **"Save"**

5. **Increase VM Window Size:**
   - **In UTM:** Make the VM window as large as possible (drag corners)
   - **Or go fullscreen:** Press `Cmd+Ctrl+F` or View → Fullscreen
   - **In Ubuntu VM:** Settings → Displays → Increase resolution to at least 1440x900

---

## Step 4: Install Ubuntu

1. **Start the VM:**
   - Select your VM
   - Click **"Play"** button (or press `Space`)

2. **Ubuntu Installer:**
   - Ubuntu installer should boot directly
   - If you see a boot menu, select **"Install Ubuntu"**
   - Press Enter

3. **Follow Installation Wizard:**
   - Choose language
   - Choose keyboard layout
   - Select **"Normal installation"**
   - ✅ Check **"Install third-party software"**
   - Choose **"Erase disk and install Ubuntu"** (safe - only affects virtual disk)
   - Set your username and password
   - Wait for installation (15-20 minutes)

4. **After Installation:**
   - Click "Restart Now"
   - Remove ISO: VM → Settings → Drives → Remove ISO
   - VM will boot into Ubuntu

---

## Step 5: Install SDK Manager

1. **In Ubuntu VM, open Terminal:**
   - Press `Ctrl+Alt+T`

2. **Download SDK Manager:**
   
   **Option A: Download from NVIDIA Website (Recommended)**
   ```bash
   # Open Firefox browser in Ubuntu
   firefox https://developer.nvidia.com/sdk-manager
   
   # Or use the default browser
   xdg-open https://developer.nvidia.com/sdk-manager
   ```
   - Sign in to your NVIDIA Developer account (create one if needed - it's free)
   - Navigate to SDK Manager downloads
   - Download the latest `.deb` file for Linux
   - Save it to your Downloads folder
   
   **Option B: Download via Command Line (if you have the direct link)**
   ```bash
   # First, install wget if not already installed
   sudo apt update
   sudo apt install -y wget
   
   # Visit https://developer.nvidia.com/sdk-manager to get the latest download link
   # Then use wget with the correct URL
   # Example (update with actual URL from website):
   # wget <actual-download-url-from-nvidia-website>
   ```

3. **Install SDK Manager:**

   **If you downloaded a .deb file:**
   ```bash
   # Navigate to Downloads
   cd ~/Downloads
   
   # Install the .deb file
   sudo apt install -y ./jetpack-sdk-manager_*.deb
   
   # Fix dependency errors if any
   sudo apt install -f
   
   # Launch SDK Manager
   sdkmanager
   ```

3. **Flash Jetson Using SDK Manager:**
   
   **📖 For complete detailed instructions, see: [SDK_MANAGER_WALKTHROUGH.md](../SDK_MANAGER_WALKTHROUGH.md)**
   
   **Quick Steps:**
   - **Step 01:** Login, select "Jetson Orin Nano", choose JetPack 6.2.1
   - **Step 02:** Review components, accept license, enter password
   - **Step 03:** Wait for download/install (1-2 hours), connect Jetson when prompted
   - **Step 04:** Complete setup
   
   **When connecting Jetson:**
   - Put Jetson in recovery mode (jumper method - see SDK_MANAGER_WALKTHROUGH.md)
   - Connect USB-C to Mac
   - **In UTM:**
     - Right-click VM window or click USB icon in toolbar
     - Look for "USB Device [05ac:1460]" (your Jetson)
     - Click on device → Click "Connect..." to attach to VM
     - ⚠️ **Device must show as connected**
   - Click "Refresh" in SDK Manager
   - SDK Manager will detect the board
   
   **If you get "libusb_open: No such device" error:**
   - The USB device is not connected to the VM
   - Make sure you clicked "Connect..." in UTM
   - Verify connection: Run `lsusb` in VM terminal
   - See troubleshooting in SDK_MANAGER_WALKTHROUGH.md

---

## Troubleshooting

### USB Device Not Showing
- Make sure "Enable USB Sharing" is checked in VM settings
- Try disconnecting and reconnecting USB device
- Check UTM toolbar for USB icon

### Error: "Device is not ready to flash"
**If SDK Manager detects your Jetson but shows "device is not ready to flash" error:**
- **Most common fix:** Change USB Support from "USB 3.0 (XHCI)" to "USB 2.0 (EHCI)" in VM Settings → USB
- Wait 15-20 seconds after connecting USB device to VM (let it fully initialize)
- Try a different USB port on your Mac
- Try a different USB-C cable
- **📖 For detailed troubleshooting, see: [FIX_DEVICE_NOT_READY_TO_FLASH.md](FIX_DEVICE_NOT_READY_TO_FLASH.md)**

### Performance Issues
- Increase allocated RAM and CPU cores
- Close other applications on your Mac

### Can't Boot from ISO
- Make sure ISO is properly selected in VM settings
- Try removing and re-adding the ISO

---

**UTM is much better for Apple Silicon Macs! You should have a smoother experience than with VirtualBox.**

