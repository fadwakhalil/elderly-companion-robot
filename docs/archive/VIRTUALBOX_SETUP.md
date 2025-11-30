# VirtualBox Setup Guide for Jetson Flashing

**Step-by-step guide to create Ubuntu VM in VirtualBox for flashing Jetson.**

---

## Step 1: Download Ubuntu ISO (5-10 minutes)

You have a `.torrent` file, so you need to download the actual ISO:

1. **Install a torrent client (if needed):**
   ```bash
   # On macOS, install Transmission (free)
   brew install --cask transmission
   
   # Or download from: https://transmissionbt.com/
   ```

2. **Open the torrent file:**
   - If you have `ubuntu-22.04.5-desktop-amd64.iso.torrent`, double-click it
   - If you have `ubuntu-24.04.3-desktop-amd64.iso.torrent`, that works too (see version notes below)
   - Transmission (or your torrent client) will open
   - Choose download location (e.g., `~/Downloads`)
   - Wait for download to complete
   - You should end up with: `ubuntu-XX.XX.X-desktop-amd64.iso` (about 5-6 GB)

3. **Choose Ubuntu Version:**
   - **Ubuntu 22.04.5 LTS (Recommended):**
     - Best compatibility with ROS 2 Humble (requires 22.04)
     - Works with all JetPack versions
     - Download from: https://ubuntu.com/download/desktop → "Alternative downloads" → "Past releases"
   - **Ubuntu 24.04.3 LTS:**
     - Works with JetPack 6.2+ and newer SDK Manager
     - ⚠️ ROS 2 Humble officially requires Ubuntu 22.04
     - Direct download available on main page: https://ubuntu.com/download/desktop

4. **Verify the ISO:**
   - Check file size: Should be around 5-6 GB
   - File extension should be `.iso` (not `.torrent`)

---

## Step 2: Install VirtualBox Extension Pack (2 minutes)

**Important:** You need the Extension Pack for USB 3.0 support (required for Jetson flashing).

1. **Download Extension Pack:**
   - Go to: https://www.virtualbox.org/wiki/Downloads
   - Under "VirtualBox Extension Pack", click "All supported platforms"
   - Download the `.vbox-extpack` file

2. **Install Extension Pack:**
   - Open VirtualBox (if not already open)
   - Go to: **VirtualBox → Preferences → Extensions**
   - Click the **"+"** button (or right-click → Add Package)
   - Select the downloaded `.vbox-extpack` file
   - Click **"Install"**
   - Accept the license agreement
   - Enter your Mac password when prompted

---

## Step 3: Create VirtualBox VM (5 minutes)

### 3.1 Create New Virtual Machine

1. **Open VirtualBox**
   - Launch VirtualBox from Applications

2. **Click "New" button** (or press `Cmd+N`)

3. **Basic Information:**
   - **Name:** `Ubuntu-Jetson-Flash`
   - **Machine Folder:** Leave default (or choose custom location)
   - **ISO Image:** Select your Ubuntu ISO file (e.g., `ubuntu-22.04.5-desktop-amd64.iso`)
   - **Type:** `Linux` (should auto-detect)
   - **Version:** **IMPORTANT:** Make sure it says **"Ubuntu (64-bit)"** or **"Ubuntu"** 
     - ⚠️ **NOT "Ubuntu (ARM 64-bit)"** - that's for ARM processors
     - If it shows "ARM 64-bit", click the dropdown and select **"Ubuntu (64-bit)"** or just **"Ubuntu"**
     - Your ISO is `amd64` (Intel/AMD), so you need the x86_64/AMD64 version, not ARM
   - Click **"Continue"** (or "Next")
   
   **⚠️ If you see a warning:** "VirtualBox can't install an OS from the selected ISO" - that's OK! We're installing manually anyway. Just make sure the OS Version matches your ISO (64-bit, not ARM 64-bit).

### 3.2 Allocate Memory (RAM)

1. **Set Memory Size:**
   - **4096 MB (4 GB)** minimum
   - **8192 MB (8 GB)** recommended if you have 16GB+ RAM on your Mac
   - Use the slider or type the value
   - **Green zone is safe** (yellow/orange is OK, red is too much)
   - Click **"Continue"**

### 3.3 Create Virtual Hard Disk

**What you're doing:** Creating a virtual hard drive for your VM (like a hard drive for a computer)

1. **Hard Disk:**
   - You'll see options like:
     - "Do not add a virtual hard disk"
     - "Create a virtual hard disk now" ✅ **SELECT THIS**
   - Select **"Create a virtual hard disk now"**
   - Click **"Create"** (or "Next" or "Continue" - button name varies)

2. **Hard Disk File Type:**
   - Select **"VDI (VirtualBox Disk Image)"**
   - Click **"Continue"** (or "Next" or "Create" - depends on your VirtualBox version)

3. **Storage Allocation Type:**
   - You'll see two options (may be labeled differently depending on VirtualBox version):
     - **"Dynamically allocated"** ✅ **SELECT THIS ONE** (recommended)
       - Uses disk space as needed (starts small, grows up to max size)
       - More efficient use of your Mac's storage
     - **"Fixed size"** (don't select this)
       - Allocates full size immediately
   - **Look for:** Radio buttons or checkboxes with these options
   - **Select "Dynamically allocated"**
   - Click **"Continue"** (or "Next")

4. **File Location and Size:**
   - You'll see two fields:
     - **Location/File path:** 
       - ✅ **Recommended:** Leave default location (usually `~/VirtualBox VMs/Ubuntu-Jetson-Flash/`)
       - Or click the **folder icon** (📁) to browse and choose a custom location
     - **Size:** 
       - Set to **50.00 GB** (or more if you have space)
       - Use the slider or type the number
   - Click **"Create"** (or "Finish")

**💡 Can't find these options?**
- Make sure you selected "Create a virtual hard disk now" in step 1
- The exact wording may vary by VirtualBox version
- Look for options about "allocation" or "storage type"
- If you see "Dynamically allocated" vs "Fixed size" - choose "Dynamically allocated"

**✅ VM Created!** You should see `Ubuntu-Jetson-Flash` in the VirtualBox Manager list.

**Note:** The virtual disk file will be created automatically. You don't need to create it manually - VirtualBox handles this.

---

## Step 4: Configure VM Settings (5 minutes)

### 4.1 Mount Ubuntu ISO

**Step-by-step instructions:**

1. **Open Settings:**
   - In VirtualBox Manager, **select your VM** (`Ubuntu-Jetson-Flash`) in the list (click once to highlight it)
   - Click the **"Settings"** button (gear icon) at the top, or press `Cmd+S`
   - Settings window will open

2. **Navigate to Storage:**
   - In the left sidebar of Settings, click **"Storage"**
   - You'll see storage configuration on the right

3. **Find the Empty CD Drive:**
   - Look at the storage tree on the left side of the Storage settings
   - You'll see:
     - `Controller: SATA` (with your virtual hard disk)
     - `Controller: IDE` (with an empty CD icon that says "Empty")
   - **Click on the "Empty" CD icon** under `Controller: IDE`

4. **Select the ISO File:**
   - On the right side, you'll see "Attributes" section
   - Next to "Optical Drive:", there's a **CD icon** (looks like a disc)
   - **Click that CD icon**
   - A menu will appear - select **"Choose a disk file..."**

5. **Browse and Select ISO:**
   - A file browser window will open
   - Navigate to your **Downloads folder** (or wherever you saved the Ubuntu ISO)
   - Look for your Ubuntu ISO file:
     - `ubuntu-22.04.5-desktop-amd64.iso` OR
     - `ubuntu-24.04.3-desktop-amd64.iso`
   - **Click on the ISO file** to select it
   - Click **"Open"**

6. **Verify:**
   - You should now see the ISO filename listed under `Controller: IDE` instead of "Empty"
   - The ISO is now mounted and ready!

**✅ ISO Mounted!** Don't click OK yet - we still need to enable USB 3.0.

### 4.2 Enable USB 3.0 (CRITICAL for Jetson)

**⚠️ This is essential for connecting your Jetson to the VM!**

**Step-by-step instructions:**

1. **Navigate to USB Settings:**
   - **Still in the Settings window** (don't close it yet!)
   - In the left sidebar, click **"USB"**
   - USB settings will appear on the right

2. **Enable USB Controller:**
   - At the top, you'll see **"Enable USB Controller"** checkbox
   - ✅ **Check this box** (click it to put a checkmark)

3. **Select USB 3.0:**
   - Below the checkbox, you'll see **"USB Controller:"** with a dropdown menu
   - Click the dropdown menu
   - Select **"USB 3.0 (xHCI) Controller"** from the list
     - If you don't see USB 3.0 option, make sure you installed the Extension Pack (Step 2)
     - You might see: "USB 1.1 (OHCI) Controller", "USB 2.0 (EHCI) Controller", "USB 3.0 (xHCI) Controller"
     - Choose the **USB 3.0** option

4. **Save Settings:**
   - Click **"OK"** at the bottom of the Settings window
   - Settings will close and return you to VirtualBox Manager

**✅ USB 3.0 Enabled!** Your VM is now configured to use USB 3.0, which is required for Jetson flashing.

### 4.3 Adjust System Settings (Important!)

1. **System Settings:**
   - Click **"System"** in left sidebar
   - **Motherboard tab:**
     - **Enable I/O APIC:** ✅ Check this box
     - **Firmware:** Change from "EFI" to **"BIOS"** (this allows boot order control)
       - If you see "EFI" selected, click the dropdown and select **"BIOS"**
       - This will enable the Boot Order section below
     - **Boot Order:** Now you can control this (it was greyed out before)
       - Make sure **"Optical"** is checked and at the top
       - Use the up/down arrows to move "Optical" to the top
       - Uncheck "Floppy" if you don't need it
   - **Processor tab:**
     - Processors: Set to **2-4 CPUs** (recommended)
       - **2 CPUs:** Minimum, sufficient for most operations
       - **4 CPUs:** Better performance for SDK Manager and flashing (recommended if you have 8+ cores)
       - **Don't allocate more than 50% of your Mac's CPUs** (e.g., if you have 10 cores, max 4-5 CPUs)
       - Too many CPUs can actually hurt performance
     - ✅ Check **"Enable PAE/NX"**
   - Click **"OK"**

---

## Step 5: Install Ubuntu (15-20 minutes)

### 5.0 Make VM Window Larger (Do This First!)

**Before starting installation, make the window bigger:**

1. **Start the VM:**
   - Select your VM
   - Click **"Start"** (green arrow) or press `Cmd+R`
   - VM window will open (it will be small initially)

2. **Resize the Window:**
   - **Option 1: Scaled Mode (Easiest)**
     - Press `Cmd+C` (or View → Scaled Mode)
     - Window will resize to fit your screen better
     - You can drag corners to resize further
   
   - **Option 2: Fullscreen Mode**
     - Press `Cmd+F` (or View → Fullscreen Mode)
     - VM takes over entire screen
     - Press `Cmd+F` again to exit fullscreen
   
   - **Option 3: Adjust Display Settings**
     - While VM is running, go to: **Machine → Settings → Display**
     - Increase **"Video Memory"** to 128 MB (or higher if available)
     - Set **"Scale Factor"** to 200% or higher
     - Click OK (you may need to restart VM)

3. **If window is still too small:**
   - You can install Guest Additions after Ubuntu is installed (Step 6)
   - For now, use Scaled Mode (`Cmd+C`) - this should help immediately

2. **Boot Process:**
   - **If using BIOS mode (recommended):** Ubuntu installer should boot directly from the ISO
   - **If you still see UEFI menu:** 
     - Use **arrow keys (↑↓)** to move highlight
     - Select **"Boot Manager"** (highlight it)
     - Press **Enter**
     - Select the Ubuntu ISO/CD option
     - Press Enter

3. **Ubuntu Installer Boots:**
   - You'll see the Ubuntu boot menu (purple/black screen with Ubuntu logo)
   - Select **"Install Ubuntu"** (use arrow keys if needed)
   - Press **Enter**

3. **Installation Wizard:**

   **Language Selection:**
   - Choose your language
   - Click **"Continue"**

   **Keyboard Layout:**
   - Choose your keyboard layout
   - Click **"Continue"**

   **Updates and Other Software:**
   - Select **"Normal installation"**
   - ✅ Check **"Install third-party software"** (for better hardware support)
   - Click **"Continue"**

   **Installation Type:**
   - Select **"Erase disk and install Ubuntu"**
     - ⚠️ **Don't worry!** This only erases the virtual disk, NOT your Mac's hard drive
   - Click **"Install Now"**
   - Confirm by clicking **"Continue"**

   **Where Are You?:**
   - Select your timezone on the map
   - Click **"Continue"**

   **Who Are You?:**
   - **Your name:** (e.g., `jetson-user`)
   - **Your computer's name:** (e.g., `ubuntu-jetson-flash`)
   - **Pick a username:** (e.g., `jetson`)
   - **Choose a password:** (remember this!)
   - **Confirm your password:**
   - Select **"Require my password to log in"** (recommended)
   - Click **"Continue"**

4. **Wait for Installation:**
   - Installation will take 10-20 minutes
   - You'll see a slideshow about Ubuntu features
   - **Don't close the VM window!**

5. **Installation Complete:**
   - When done, you'll see **"Installation Complete"**
   - Click **"Restart Now"**

6. **Remove Installation Media:**
   - After restart, you may see a message about removing installation media
   - In VirtualBox menu: **Devices → Optical Drives → Remove disk from virtual drive**
   - Or go to: **Machine → Settings → Storage → Remove the ISO**
   - Press Enter in the VM to continue booting

7. **Ubuntu Desktop:**
   - Ubuntu will boot to the login screen
   - Enter your password
   - You're now in Ubuntu!

---

## Step 6: Install VirtualBox Guest Additions (Optional but Recommended)

Guest Additions improve performance, enable better screen resolution, and improve USB support.

1. **In the Ubuntu VM:**
   - Go to VirtualBox menu: **Devices → Insert Guest Additions CD image**
   - A CD icon should appear on the Ubuntu desktop

2. **Open Terminal in Ubuntu:**
   - Press `Ctrl+Alt+T` or click the Terminal icon

3. **Install Guest Additions:**
   ```bash
   # Update package lists
   sudo apt update

   # Install required packages
   sudo apt install -y build-essential dkms linux-headers-$(uname -r)

   # Mount and run Guest Additions
   cd /media/*/VBox_GAs_*
   sudo ./VBoxLinuxAdditions.run

   # Reboot
   sudo reboot
   ```

4. **After Reboot:**
   - Screen resolution should be better
   - You can resize the VM window
   - USB support is improved

---

## Step 7: Test USB Connection

Before flashing Jetson, test that USB works:

1. **In VirtualBox:**
   - With VM running, go to menu: **Devices → USB**
   - You should see USB devices listed
   - When you connect a USB device to your Mac, it should appear here
   - You can click to attach it to the VM

2. **Test with a USB drive:**
   - Plug a USB drive into your Mac
   - In VirtualBox: **Devices → USB → [Your USB Device]**
   - It should appear in Ubuntu's file manager

---

## ✅ You're Ready!

Your Ubuntu VM is now set up and ready for:
- Installing SDK Manager (next step in HARDWARE_SETUP_GUIDE.md)
- Flashing your Jetson

**Next Steps:**
- Go back to [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)
- Continue with **Step A.2: Install SDK Manager in VM**

---

## Troubleshooting

### VM Won't Start
- Check if virtualization is enabled in your Mac's BIOS/UEFI (usually enabled by default on Mac)
- Try reducing allocated RAM
- Check VirtualBox logs: **Help → Show Log**

### USB Not Working
- Make sure Extension Pack is installed
- Check USB controller is set to USB 3.0 in VM settings
- Try detaching and reattaching USB device
- On macOS, you may need to allow VirtualBox in System Preferences → Security & Privacy

### Screen Resolution Too Small
- Install Guest Additions (Step 6 above)
- Or manually set resolution in Ubuntu: Settings → Displays

### Installation Stuck
- Make sure you have enough disk space on your Mac
- Try increasing VM's allocated RAM
- Check if ISO file is complete (should be ~5-6 GB)

---

**Need Help?** Check the main [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md) for more details.

