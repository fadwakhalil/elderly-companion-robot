# Bootable Ubuntu USB Flash Method

**Create a bootable Ubuntu USB drive and flash Jetson directly from it. This bypasses all VM USB passthrough issues!**

---

## ✅ Why This Method Works

- **No VM USB passthrough issues** - Direct hardware access
- **Native Ubuntu environment** - More reliable than VM
- **Doesn't affect your Mac** - Uses "Try Ubuntu" mode (no installation)
- **Once flashed, develop via SSH** - Keep using your Mac normally

---

## Phase 1: Create Bootable Ubuntu USB

### Step 1.1: Download Required Files

**On your Mac:**

1. **Download Ubuntu 22.04 LTS:**
   - Go to: https://releases.ubuntu.com/22.04/
   - Download: `ubuntu-22.04.5-desktop-amd64.iso` (recommended)
   - Or: `ubuntu-22.04.3-desktop-amd64.iso`
   - File size: ~5-6 GB
   - ⚠️ **Note:** You can reuse the ISO you already downloaded for your VM!

2. **Download Balena Etcher:**
   - Go to: https://www.balena.io/etcher/
   - Download for macOS
   - Free and open source

### Step 1.2: Prepare USB Drive

**Requirements:**
- **16GB or larger USB 3.0 drive** (faster is better)
- **Backup any important data** - the USB drive will be completely erased

### Step 1.3: Create Bootable USB with Balena Etcher

1. **Insert USB drive** into your Mac

2. **Open Balena Etcher**

3. **Three-step process:**
   - **Step 1:** Click **"Flash from file"** → Select the Ubuntu ISO you downloaded
   - **Step 2:** Click **"Select target"** → Choose your USB drive
   - **Step 3:** Click **"Flash!"** → Wait for completion (10-20 minutes)

4. **Verify the flash:**
   - Etcher will automatically validate the write
   - Wait for "Flash Complete!" message

**⚠️ Troubleshooting balenaEtcher Errors:**

If you get "The writer process ended unexpectedly" error:

1. **Check ISO file integrity:**
   ```bash
   # Verify the ISO file size (should be ~5-6 GB)
   ls -lh ~/Downloads/ubuntu-*.iso
   
   # Check if file is corrupted (this takes a few minutes)
   # Ubuntu ISOs have checksums - verify if possible
   ```

2. **Try a different USB drive:**
   - Some USB drives are incompatible
   - Try a different brand/model
   - Make sure it's USB 3.0 (faster, more reliable)

3. **Reformat USB drive first:**
   ```bash
   # In Terminal, format the USB drive
   diskutil list  # Find your USB drive (e.g., /dev/disk2)
   diskutil eraseDisk FAT32 USBDRIVE /dev/diskX  # Replace X with your disk number
   ```

4. **Use command line method instead** (see below) - often more reliable

5. **Re-download ISO if corrupted:**
   - Delete the current ISO
   - Re-download from Ubuntu website
   - Make sure download completes fully

**Alternative: Command Line Method (More Reliable)**

If you prefer terminal:

```bash
# 1. Identify your USB drive (BE CAREFUL - this will ERASE the target disk)
diskutil list

# 2. Look for your USB drive (usually /dev/disk2 or /dev/disk3)
#    Example output:
#    /dev/disk2 (external, physical):
#       #:                       TYPE NAME                    SIZE       IDENTIFIER
#       0:     FDisk_partition_scheme                        *16.0 GB    disk2
#       1:                 DOS_FAT_32 USB DRIVE               16.0 GB    disk2s1

# 3. Unmount the USB drive (replace X with your disk number, e.g., disk2)
sudo diskutil unmountDisk /dev/diskX

# 4. Create bootable USB (this takes 10-20 minutes)
#    Replace X with your disk number and path with your ISO location
sudo dd if=~/Downloads/ubuntu-22.04.5-desktop-amd64.iso of=/dev/diskX bs=1m

# 5. Wait for completion (you'll see progress)
# 6. Eject the USB when done
sudo diskutil eject /dev/diskX
```

---

## Phase 2: Boot from USB and Setup

### Step 2.1: Boot from USB Drive

1. **Shutdown your Mac completely**

2. **Insert the Ubuntu USB drive**

3. **Power on Mac** and immediately hold the **Option (⌥) key**

4. **You'll see boot options:**
   - Select **"EFI Boot"** (orange icon, usually labeled "EFI Boot" or "Windows")
   - This is your Ubuntu USB drive

5. **Ubuntu will start loading:**
   - You'll see the Ubuntu logo
   - Wait for the boot menu (may take 1-2 minutes)

### Step 2.2: Try Ubuntu Without Installing

1. **When Ubuntu boots, select "Try Ubuntu"** (NOT "Install Ubuntu")
   - This gives you a full Ubuntu desktop
   - **Doesn't touch your Mac's hard drive** - completely safe
   - You can use it like a normal Ubuntu system

2. **Connect to WiFi:**
   - Click network icon in top-right
   - Select your WiFi network
   - Enter password
   - **Wait for connection** - you should see WiFi icon change to show signal strength

3. **Verify internet connection:**
   ```bash
   # In terminal, test internet connection
   ping -c 3 8.8.8.8
   # If this works, internet is connected but DNS might be the issue
   
   # Test DNS
   ping -c 3 google.com
   # If this fails, DNS is the problem
   ```

4. **If DNS is the issue, fix it:**
   ```bash
   # Edit DNS settings
   sudo nano /etc/resolv.conf
   # Add these lines:
   nameserver 8.8.8.8
   nameserver 8.8.4.4
   # Save: Ctrl+X, then Y, then Enter
   ```

5. **Wait for Ubuntu to fully load:**
   - Desktop should appear
   - All features work normally

### Step 2.3: Setup Ubuntu Environment for Flashing

**In the live Ubuntu session, open Terminal** (`Ctrl+Alt+T`) and run:

```bash
# Update package list (important - do this first!)
sudo apt update

# Enable universe repository (needed for some packages)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install essential tools for Jetson flashing
sudo apt install -y \
    python3 \
    python3-pip \
    wget \
    curl \
    lsb-release \
    lbzip2 \
    libxml2-utils \
    qemu-user-static \
    usbutils \
    git \
    firefox

# Note: Some packages might already be installed in the live session
# If you get "already installed" messages, that's fine - continue
```

**If packages aren't available, check what's already installed:**

```bash
# Check what's already installed (many packages come pre-installed in live session)
python3 --version
wget --version
curl --version
git --version
firefox --version
```

**For SDK Manager, you only need these essentials:**

```bash
# 1. Update and enable repositories
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt update

# 2. Install only essential packages (skip the ones that fail)
sudo apt install -y python3
sudo apt install -y wget  # Usually pre-installed
sudo apt install -y firefox  # For downloading SDK Manager

# 3. Try to install pip separately if needed
sudo apt install -y python3-venv python3-setuptools
python3 -m ensurepip --upgrade  # This installs pip without the package
```

**If packages still fail, that's OK!** Many tools are pre-installed in Ubuntu live session. You can proceed to download SDK Manager with just `wget` or `firefox`.

---

## Phase 3: Download and Install SDK Manager

### Step 3.1: Download SDK Manager

**In Ubuntu Terminal:**

```bash
# Option A: Download via browser (easier)
# Open Firefox (should be installed)
firefox https://developer.nvidia.com/sdk-manager

# Sign in to your NVIDIA Developer account
# Download the .deb file for Ubuntu 22.04
# Save to Downloads folder

# Option B: Download via command line (if you have direct link)
# Check NVIDIA website for latest download link
cd ~/Downloads
# Example (update with actual URL from website):
# wget <actual-download-url-from-nvidia-website>
```

### Step 3.2: Install SDK Manager

```bash
# Navigate to Downloads
cd ~/Downloads

# Install the .deb file (replace with actual filename)
sudo apt install -y ./sdkmanager_*.deb

# Or if you downloaded a tarball:
# tar -xzf sdkmanager-*.tar.gz
# cd sdkmanager-*
# sudo apt install -y ./sdkmanager_*.deb

# Fix any dependency issues
sudo apt install -f

# Verify installation
which sdkmanager
sdkmanager --version
```

---

## Phase 4: Flash Jetson Orin Nano

### Step 4.1: Prepare Jetson for Flashing

**Physical setup:**

1. **Connect Jetson to official 45W power adapter**
2. **Connect USB-C cable** from Jetson to Mac (use USB-C port on Mac if available)
3. **Ensure Jetson is powered OFF** (no lights)
4. **Insert SD card** into Jetson (if not already inserted)

### Step 4.2: Enter Recovery Mode (Jumper Method)

**⚠️ CRITICAL: For Jetson Orin Nano, use the JUMPER method (not button method):**

1. **Power off Jetson** (disconnect power adapter)

2. **Locate J40 header** on the Jetson carrier board
   - Look for pins labeled "9" and "10"
   - Usually near the edge of the board

3. **Place jumper on pins 9-10:**
   - Use a small jumper wire or paperclip
   - Connect pin 9 to pin 10
   - This puts Jetson in recovery mode

4. **Connect USB-C cable** from Jetson to Mac

5. **Connect power adapter** to Jetson

6. **Wait 5 seconds** - Jetson should power on

7. **Remove jumper** from pins 9-10
   - Jetson is now in recovery mode

8. **Verify in Ubuntu Terminal:**
   ```bash
   lsusb | grep -i nvidia
   # Should show: Bus XXX Device XXX: ID 0955:7523 NVIDIA Corp. APX
   ```

### Step 4.3: Use SDK Manager to Flash

**Now use SDK Manager (same as VM method):**

1. **Launch SDK Manager:**
   ```bash
   sdkmanager
   ```

2. **Follow SDK Manager walkthrough:**
   - **Step 01:** Select "Jetson Orin Nano [8GB developer kit version]" (NOT 4GB!)
   - **Step 02:** Review components, accept license
   - **Step 03:** Download and install (30-60 minutes)
   - **Step 04:** Flash Configuration
     - Select "Manual Setup - Jetson Orin Nano [8GB developer kit version]"
     - Enter username/password
     - Click "Refresh" - should detect Jetson
     - Click "Flash" - wait 20-40 minutes

3. **📖 For detailed SDK Manager steps, see: [SDK_MANAGER_WALKTHROUGH.md](SDK_MANAGER_WALKTHROUGH.md)**

### Step 4.4: Alternative - Manual Flash (Advanced)

**If SDK Manager still has issues, you can use manual flash:**

```bash
# 1. Download JetPack Linux for Tegra (L4T)
#    Go to: https://developer.nvidia.com/embedded/jetson-linux
#    Download: JetPack 6.2.1 for Jetson Orin Nano
#    File: jetson_linux_r36.2.0_aarch64.tbz2 (or similar)

# 2. Extract
cd ~/Downloads
tar xf jetson_linux_r36.2.0_aarch64.tbz2
cd Linux_for_Tegra/

# 3. Flash Jetson (specify 8GB variant)
sudo ./flash.sh jetson-orin-nano-devkit internal

# Wait for completion - you'll see:
# *** Flashing completed successfully ***
```

**⚠️ Important:** Make sure to use the correct flash command for 8GB variant:
- `jetson-orin-nano-devkit` - This is for the 8GB developer kit
- The command may vary by JetPack version - check NVIDIA documentation

### Step 4.5: Post-Flash Setup

1. **Disconnect USB cable** from Jetson
2. **Power cycle Jetson** (turn off and on)
3. **Connect monitor, keyboard, and mouse** to Jetson
4. **Follow Ubuntu setup wizard** on Jetson itself
5. **Note the IP address** for future SSH access

---

## Phase 5: Setup Development Environment on Jetson

### Step 5.1: Initial Jetson Setup

**On the Jetson itself (with monitor/keyboard):**

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y \
    curl \
    wget \
    git \
    python3-pip \
    python3-venv \
    build-essential

# Install Docker (for your containerized AI services)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Log out and back in for group changes
```

### Step 5.2: Install ROS 2 on Jetson

```bash
# Setup locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Phase 6: Return to macOS and Setup Remote Development

### Step 6.1: Shutdown Ubuntu Live Session

1. **Shutdown the Ubuntu live session:**
   - Click power icon in top-right
   - Select "Shut Down"
   - Or just power off Mac

2. **Remove USB drive**

3. **Restart Mac** - it will boot back to macOS (your Mac is unchanged!)

### Step 6.2: Setup SSH to Jetson

**On your Mac:**

```bash
# SSH to Jetson (replace with actual IP and username)
ssh username@jetson-ip-address

# Setup passwordless SSH
ssh-copy-id username@jetson-ip-address

# Now you can SSH without password
ssh username@jetson-ip-address
```

### Step 6.3: Transfer Your Project Files

**From your Mac to Jetson:**

```bash
# Copy your project files
scp -r ~/Documents/ops/ai-project/elderly_companion username@jetson-ip-address:~/

# Or use rsync for better progress
rsync -avz --progress ~/Documents/ops/ai-project/elderly_companion username@jetson-ip-address:~/
```

---

## Expected Time Investment

- **Creating bootable USB:** 20-30 minutes
- **Booting Ubuntu and setup:** 5-10 minutes
- **Downloading SDK Manager:** 5-10 minutes
- **Installing SDK Manager:** 2-5 minutes
- **Flashing Jetson:** 30-60 minutes
- **Jetson setup:** 20-30 minutes
- **Total:** ~2-3 hours

---

## Troubleshooting Tips

### Jetson Not Detected in Recovery Mode

1. **Try different USB cable:**
   - Must support data transfer (not just charging)
   - Try a shorter, high-quality cable

2. **Try different USB port on Mac:**
   - Avoid USB hubs
   - Try connecting directly to Mac
   - Try a different USB-C port

3. **Verify recovery mode:**
   ```bash
   lsusb | grep -i nvidia
   # Should show: ID 0955:7523 NVIDIA Corp. APX
   ```

4. **Ensure proper power supply:**
   - Use official 45W power adapter
   - Make sure power adapter is connected

### Flash Fails

1. **Ensure Jetson stays in recovery mode:**
   - Don't remove jumper too early
   - Don't disconnect cables during flashing

2. **Use stable internet connection:**
   - Downloads need reliable connection
   - Consider using Ethernet if WiFi is unstable

3. **Check disk space:**
   ```bash
   df -h
   # Make sure you have at least 50GB free
   ```

### Boot Issues After Flash

1. **Ensure proper display connection:**
   - Connect monitor via HDMI
   - Check display settings

2. **Check power supply:**
   - Ensure 45W power adapter is adequate
   - Check power LED on Jetson

3. **Verify SD card:**
   - Make sure SD card is properly inserted
   - Try a different SD card if available

---

## Advantages of This Method

✅ **No VM USB passthrough issues** - Direct hardware access  
✅ **Native Ubuntu environment** - More reliable  
✅ **Doesn't affect your Mac** - Uses live session  
✅ **Can reuse USB drive** - After flashing, format and use normally  
✅ **Once flashed, develop via SSH** - Keep using your Mac normally  

---

## Comparison to Other Methods

| Method | Reliability | Time | Complexity |
|--------|------------|------|------------|
| **Bootable USB (This)** | ⭐⭐⭐⭐⭐ | 2-3 hours | Medium |
| VM with SDK Manager | ⭐⭐ | 2-3 hours | High (USB issues) |
| Windows/Linux Computer | ⭐⭐⭐⭐⭐ | 1-2 hours | Low |
| Pre-flashed SD Card | ⭐⭐⭐⭐⭐ | 0 hours | Low (but costs money) |

---

## Next Steps

After successfully flashing:

1. **Follow:** [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md) for Jetson setup
2. **Deploy your AI services** to Jetson
3. **Develop remotely** via SSH from your Mac

---

**This method bypasses all VM USB passthrough issues and gives you a clean, reliable flashing environment. Once flashed, you'll develop on the Jetson directly via SSH from your Mac!**

**Ready to proceed? Start with Phase 1 and let me know when you have the bootable USB created!** 🚀

