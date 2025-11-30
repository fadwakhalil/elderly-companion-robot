# Alternative Flashing Methods - When VM USB Passthrough Fails

**If you've tried everything and still get "device is not ready for flash" error, try these alternatives.**

---

## 🎯 Quick Decision Guide

**Choose the best option for your situation:**

- ✅ **Want to avoid VM issues completely?** → **Option 0** (Bootable USB - Recommended!)
- ✅ **Have access to a Windows/Linux computer?** → **Option 1** (Easiest)
- ✅ **Comfortable with command line?** → **Option 2** (Command-line tools)
- ✅ **Have Parallels Desktop?** → **Option 3** (Better USB support)
- ✅ **Want to buy pre-flashed SD card?** → **Option 4** (Easiest but costs money)
- ✅ **Can use another Mac with VirtualBox?** → **Option 5** (If Intel Mac)

---

## Option 0: Bootable Ubuntu USB (Recommended - Bypasses All VM Issues!)

**Create a bootable Ubuntu USB drive and flash Jetson directly from it. This is the best alternative to VM!**

### Why This Works:

- ✅ **No VM USB passthrough issues** - Direct hardware access
- ✅ **Native Ubuntu environment** - More reliable than VM
- ✅ **Doesn't affect your Mac** - Uses "Try Ubuntu" mode (no installation)
- ✅ **Once flashed, develop via SSH** - Keep using your Mac normally

### Steps:

1. **Create bootable Ubuntu USB:**
   - Download Ubuntu 22.04 ISO (you can reuse your VM ISO!)
   - Use Balena Etcher to create bootable USB
   - Takes 20-30 minutes

2. **Boot Mac from USB:**
   - Hold Option key on boot
   - Select "EFI Boot" (your USB drive)
   - Choose "Try Ubuntu" (doesn't install on Mac)

3. **Install SDK Manager in Ubuntu:**
   - Open terminal
   - Download and install SDK Manager
   - Same process as VM

4. **Flash Jetson:**
   - Put Jetson in recovery mode
   - Connect USB-C cable
   - Use SDK Manager to flash
   - Direct hardware access - no VM issues!

5. **Return to macOS:**
   - Shutdown Ubuntu
   - Remove USB drive
   - Mac boots back to macOS (unchanged)

6. **Develop via SSH:**
   - SSH to Jetson from your Mac
   - Transfer files via scp/rsync
   - Best of both worlds!

**📖 For complete step-by-step guide, see: [BOOTABLE_USB_FLASH_METHOD.md](BOOTABLE_USB_FLASH_METHOD.md)**

**This is the recommended alternative if VM USB passthrough isn't working!**

---

## Option 1: Use a Windows/Linux Computer (Easiest if Available)

**This is the most reliable method. USB passthrough works much better on native Windows/Linux.**

### Steps:

1. **Find a Windows or Linux computer:**
   - Friend's computer
   - Work computer
   - Library computer (if allowed)
   - Dual-boot your Mac (if you have Windows installed)

2. **On that computer:**
   - Download and install SDK Manager from: https://developer.nvidia.com/sdk-manager
   - Follow the standard SDK Manager walkthrough
   - USB passthrough works natively (no VM issues)

3. **Flash the SD card:**
   - Put Jetson in recovery mode
   - Connect USB-C cable
   - Use SDK Manager to flash
   - Takes 30-60 minutes

4. **Insert SD card into Jetson:**
   - After flashing completes, remove SD card
   - Insert into Jetson
   - Power on Jetson
   - Complete Ubuntu setup

**This method avoids all VM USB passthrough issues!**

---

## Option 2: Use Command-Line Tools (Advanced)

**Use `jetson-flash` tool directly on macOS (bypasses SDK Manager GUI).**

### Prerequisites:

```bash
# Install Homebrew if you don't have it
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python 3
brew install python3

# Install jetson-flash
pip3 install jetson-flash
```

### Download JetPack Image:

**You need to download the JetPack image first. Two options:**

#### Option A: Download via SDK Manager in VM (Download Only)

1. **In your Ubuntu VM:**
   - Open SDK Manager
   - Go through Step 01-03
   - Let it download JetPack (but don't try to flash)
   - Downloads are saved to: `/home/fadwa/nvidia/nvidia_sdk/`
   - Copy the downloaded files to your Mac

2. **Find the image:**
   - Look for files like: `jetson-orin-nano-devkit-*.img` or similar
   - Copy to your Mac

#### Option B: Download from NVIDIA (If Available)

- Check NVIDIA Developer website for direct JetPack image downloads
- Some versions may be available as direct downloads

### Flash Using jetson-flash:

```bash
# Put Jetson in recovery mode (jumper method)
# Connect USB-C cable to Mac

# Flash the Jetson
jetson-flash --device jetson-orin-nano-devkit --flash-all

# Or if you have the image file:
jetson-flash --device jetson-orin-nano-devkit --image /path/to/jetpack-image.img
```

**Note:** This method may have limitations on macOS. It's worth trying, but Option 1 (Windows/Linux) is more reliable.

---

## Option 3: Try Parallels Desktop (If You Have It)

**Parallels has better USB passthrough support than UTM/VirtualBox.**

### Steps:

1. **Install Parallels Desktop** (if you have it)

2. **Create Ubuntu VM:**
   - Similar to UTM setup
   - But Parallels handles USB passthrough better

3. **USB Settings:**
   - Parallels → Configure → Hardware → USB & Bluetooth
   - Enable "Share Bluetooth devices with Mac" (optional)
   - USB 3.0 support is usually more stable in Parallels

4. **Connect Jetson:**
   - Put Jetson in recovery mode
   - Parallels should automatically detect it
   - Or manually connect via Parallels menu

5. **Use SDK Manager:**
   - Follow standard SDK Manager walkthrough
   - USB passthrough should work better than UTM

**Note:** Parallels is paid software (~$100/year). Only use this if you already have it.

---

## Option 4: Buy Pre-flashed SD Card (Easiest but Costs Money)

**Some vendors sell Jetson with pre-flashed SD cards.**

### Options:

1. **Check with your Jetson vendor:**
   - Some sellers offer pre-flashed SD cards
   - Usually costs $20-50 extra

2. **Order from NVIDIA partners:**
   - Some authorized resellers offer this service

3. **Ask someone to flash it:**
   - If you know someone with a Windows/Linux computer
   - They can flash the SD card for you
   - Then you just insert it into Jetson

**This is the easiest option if you're willing to pay or have someone help.**

---

## Option 5: Try VirtualBox (May Work on Apple Silicon)

**VirtualBox might have different USB passthrough behavior than UTM. Worth trying if UTM isn't working.**

### Important Notes:

- **Apple Silicon Macs:** VirtualBox has limited x86_64 emulation support, but USB passthrough might work differently
- **Intel Macs:** VirtualBox works well and may be better than UTM for USB passthrough
- **Worth trying:** Even on Apple Silicon, VirtualBox's USB handling might work where UTM fails

### Steps:

1. **Install VirtualBox:**
   - Download from: https://www.virtualbox.org/
   - Install VirtualBox Extension Pack (required for USB 3.0)

2. **Create Ubuntu VM:**
   - See [VIRTUALBOX_SETUP.md](VIRTUALBOX_SETUP.md) for detailed instructions

3. **USB Settings:**
   - VM Settings → USB
   - Enable USB 3.0
   - Add USB device filter for Jetson

4. **Try flashing:**
   - USB passthrough may work better on Intel Macs
   - Follow SDK Manager walkthrough

**Note:** This only works on Intel Macs. Apple Silicon Macs have limited VirtualBox support.

---

## Option 6: Try Different UTM USB Configuration

**One more UTM-specific workaround to try:**

### Advanced UTM USB Settings:

1. **Stop VM**

2. **VM Settings → USB:**
   - Try "USB 1.1 (UHCI)" instead of USB 2.0 or 3.0
   - This is the slowest but most compatible option

3. **VM Settings → QEMU:**
   - Add custom QEMU arguments:
     ```
     -device qemu-xhci,id=xhci
     -device usb-host,vendorid=0x0955,productid=0x7523
     ```
   - This forces USB passthrough for the specific Jetson device ID

4. **Restart VM and try again**

**This is a last resort for UTM. Option 1 (Windows/Linux) is still recommended.**

---

## Option 7: Use Docker on macOS (Experimental)

**Some users have had success with Docker-based flashing tools.**

### Steps:

```bash
# Install Docker Desktop for Mac
# Download from: https://www.docker.com/products/docker-desktop

# Run Jetson flashing in Docker container
# (This requires finding or creating a Docker image with flashing tools)
```

**Note:** This is experimental and may not work. Option 1 is still recommended.

---

## 🎯 Recommended Approach

**Based on your situation, I recommend:**

1. **First choice:** **Option 0** - Bootable Ubuntu USB ⭐
   - Bypasses all VM USB passthrough issues
   - Direct hardware access
   - Doesn't affect your Mac
   - Takes 2-3 hours total
   - **Best alternative to VM method!**

2. **Second choice:** **Option 1** - Use a Windows/Linux computer
   - Most reliable if you have access
   - No VM USB passthrough issues
   - Takes 30-60 minutes

3. **Third choice:** **Option 4** - Pre-flashed SD card
   - Easiest if you can get one
   - Costs money but saves time

4. **Fourth choice:** **Option 2** - Command-line tools
   - If you're comfortable with terminal
   - May have limitations on macOS

---

## Why VM USB Passthrough Fails

**Common reasons:**

- **UTM/VirtualBox USB emulation limitations** on macOS
- **Apple Silicon Macs** have additional USB passthrough challenges
- **USB 3.0/2.0 timing issues** in virtualized environments
- **Jetson recovery mode** requires very specific USB timing that VMs struggle with

**Native Windows/Linux doesn't have these issues** - that's why Option 1 is recommended.

---

## Summary

**If VM USB passthrough isn't working after trying everything:**

✅ **Best solution:** Use a Windows/Linux computer (Option 1)  
✅ **Easiest solution:** Buy pre-flashed SD card (Option 4)  
✅ **Advanced solution:** Try command-line tools (Option 2)  

**The VM approach works for many people, but USB passthrough can be finicky. Don't feel bad if you need to use an alternative method!**

---

**Need help with any of these options? Let me know which one you'd like to try!**

