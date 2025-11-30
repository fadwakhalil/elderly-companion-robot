# Fresh VM Start - Complete Cleanup Guide

**Complete guide to wipe out your existing VM and start from scratch.**

---

## Step 1: Delete the VM (Choose Your Software)

### Option A: UTM (Apple Silicon Mac)

1. **Open UTM**
   - Launch UTM from Applications

2. **Delete the VM:**
   - Select your VM in the UTM window (e.g., "Ubuntu-Jetson-Flash")
   - Right-click → **"Delete"** (or press `Delete` key)
   - **IMPORTANT:** When prompted, choose **"Delete VM and all data"** (not just "Remove from Library")
   - This deletes the VM disk file and all data

3. **Verify deletion:**
   - The VM should disappear from UTM's VM list
   - If it's still there, select it and press `Cmd+Delete` to force delete

4. **Clean up VM files (optional but recommended):**
   ```bash
   # UTM stores VMs in:
   ~/Library/Containers/com.utmapp.UTM/Data/Documents/
   
   # Check if any VM files remain:
   ls -la ~/Library/Containers/com.utmapp.UTM/Data/Documents/
   
   # If you see any .utm files, delete them:
   rm -rf ~/Library/Containers/com.utmapp.UTM/Data/Documents/*.utm
   ```

---

### Option B: VirtualBox

1. **Open VirtualBox**

2. **Delete the VM:**
   - Select your VM in the VirtualBox Manager
   - Right-click → **"Remove"**
   - **IMPORTANT:** Choose **"Delete all files"** (not just "Remove from list")
   - This deletes the VM disk file (.vdi) and all data

3. **Clean up VirtualBox files (optional):**
   ```bash
   # VirtualBox stores VMs in:
   ~/VirtualBox VMs/
   
   # Check if any VM folders remain:
   ls -la ~/VirtualBox\ VMs/
   
   # If you see any VM folders, delete them:
   rm -rf ~/VirtualBox\ VMs/Ubuntu-Jetson-Flash
   ```

---

## Step 2: Clean Up Downloaded Files (Optional)

**If you want to re-download everything fresh:**

### A. SDK Manager Downloads Folder

**⚠️ Safe to delete - SDK Manager will re-download everything!**

The SDK Manager downloads folder contains all the JetPack components that were downloaded. If you delete it, SDK Manager will re-download everything when you run it again (which is what you want for a fresh start).

**Default location:**
- `/home/YOUR_USERNAME/Downloads/nvidia/sdkm_downloads`
- Or: `~/Downloads/nvidia/sdkm_downloads`

**To delete:**

```bash
# In your Ubuntu VM terminal:

# 1. Check if the folder exists:
ls -la ~/Downloads/nvidia/sdkm_downloads

# 2. Check the size (it's usually 10-15 GB):
du -sh ~/Downloads/nvidia/sdkm_downloads

# 3. Delete the entire folder:
rm -rf ~/Downloads/nvidia/sdkm_downloads

# 4. Verify it's deleted:
ls -la ~/Downloads/nvidia/
# The sdkm_downloads folder should be gone
```

**What happens when you delete it:**
- ✅ SDK Manager will re-download all JetPack components (10-15 GB)
- ✅ This ensures you get fresh, complete downloads
- ✅ No corrupted or missing files
- ⏱️ Takes 30-60 minutes to re-download (depending on internet speed)

**Note:** You can also delete the SDK installation folder if you want a completely fresh start:
```bash
# SDK installation folder (usually ~/nvidia/nvidia_sdk)
# This is where SDK Manager installs/extracts the downloaded files
rm -rf ~/nvidia/nvidia_sdk
```

### B. SDK Manager .deb Installer File

```bash
# Check for SDK Manager installer (usually in Downloads):
ls -la ~/Downloads/*sdkmanager*.deb
ls -la ~/Downloads/*jetpack*.deb

# Delete if you want to re-download:
rm -f ~/Downloads/*sdkmanager*.deb
rm -f ~/Downloads/*jetpack*.deb
```

### B. Ubuntu ISO (Keep This!)

**⚠️ DON'T delete the Ubuntu ISO** - you can reuse it:
- `ubuntu-22.04.5-desktop-amd64.iso` (or similar)
- This is a large file (5-6 GB) and takes time to download
- You can reuse it for the new VM

**Only delete if:**
- The ISO file is corrupted
- You want a different Ubuntu version
- You have plenty of bandwidth/time to re-download

---

## Step 3: Verify Clean State

**Before creating a new VM, verify everything is clean:**

### For UTM:
```bash
# Check UTM VM directory is empty (or only has VMs you want to keep):
ls -la ~/Library/Containers/com.utmapp.UTM/Data/Documents/
```

### For VirtualBox:
```bash
# Check VirtualBox VMs directory:
ls -la ~/VirtualBox\ VMs/
```

---

## Step 4: Create Fresh VM

**Now follow the complete setup guide from scratch:**

### If using UTM:
👉 **Follow:** [UTM_SETUP.md](UTM_SETUP.md)

### If using VirtualBox:
👉 **Follow:** [VIRTUALBOX_SETUP.md](VIRTUALBOX_SETUP.md)

---

## Step 5: Fresh Installation Checklist

**When creating the new VM, make sure to:**

1. ✅ **Download fresh Ubuntu ISO** (if you deleted it)
   - Ubuntu 22.04.5 LTS (recommended)
   - Verify file size: ~5-6 GB
   - Verify file extension: `.iso` (not `.torrent`)

2. ✅ **Create VM with correct settings:**
   - **UTM:** Select "Emulate" (not "Virtualize") for x86_64
   - **Memory:** At least 4 GB (8 GB recommended)
   - **CPU Cores:** At least 2 (4 recommended)
   - **Disk Size:** At least 50 GB

3. ✅ **Install Ubuntu completely:**
   - Don't skip any installation steps
   - Wait for installation to fully complete
   - Reboot when prompted

4. ✅ **Install all required packages:**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   sudo apt install -y wget curl lsb-release
   ```

5. ✅ **Download SDK Manager fresh:**
   - Go to: https://developer.nvidia.com/sdk-manager
   - Download the latest `.deb` file for Ubuntu 22.04
   - Or download the tarball and extract it

6. ✅ **Install SDK Manager:**
   ```bash
   # If you have a .deb file:
   sudo apt install ./sdkmanager_*.deb
   
   # If you have a tarball:
   tar -xzf sdkmanager-*.tar.gz
   cd sdkmanager-*
   sudo apt install ./sdkmanager_*.deb
   ```

7. ✅ **Verify SDK Manager installation:**
   ```bash
   which sdkmanager
   sdkmanager --version
   ```

---

## Common Issues to Avoid

### ❌ Don't skip Ubuntu installation steps
- Complete the full installation
- Don't try to use "Try Ubuntu" mode

### ❌ Don't use corrupted or incomplete downloads
- Verify ISO file size matches expected size
- Verify SDK Manager download completed fully

### ❌ Don't forget USB settings
- **UTM:** Enable "Share USB devices from host" in USB settings
- **VirtualBox:** Install Extension Pack for USB 3.0 support

### ❌ Don't rush the process
- Let downloads complete fully
- Let installations finish completely
- Reboot when prompted

---

## Quick Start Commands

**After creating fresh VM and installing Ubuntu:**

```bash
# 1. Update system
sudo apt update && sudo apt upgrade -y

# 2. Install basic tools
sudo apt install -y wget curl lsb-release git

# 3. Download SDK Manager (check latest version on NVIDIA website)
# Option A: Direct download (if available)
wget https://developer.nvidia.com/downloads/sdk-manager-ubuntu-22-04

# Option B: Download via browser in VM
# Go to: https://developer.nvidia.com/sdk-manager
# Download the .deb file for Ubuntu 22.04

# 4. Install SDK Manager
sudo apt install ./sdkmanager_*.deb

# 5. Verify installation
sdkmanager --version
```

---

## Need Help?

- **VM Creation Issues:** See [UTM_SETUP.md](UTM_SETUP.md) or [VIRTUALBOX_SETUP.md](VIRTUALBOX_SETUP.md)
- **SDK Manager Issues:** See [SDK_MANAGER_WALKTHROUGH.md](SDK_MANAGER_WALKTHROUGH.md)
- **USB Connection Issues:** See [FIX_DEVICE_NOT_READY_TO_FLASH.md](FIX_DEVICE_NOT_READY_TO_FLASH.md)

---

**Ready to start fresh? Follow the steps above, then proceed with your chosen VM setup guide!**

