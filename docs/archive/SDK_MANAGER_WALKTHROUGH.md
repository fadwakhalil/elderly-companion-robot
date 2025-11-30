# SDK Manager Complete Walkthrough

**Detailed step-by-step guide for using NVIDIA SDK Manager to flash your Jetson.**

---

## Launching SDK Manager

```bash
sdkmanager
```

**Note:** If you get a resolution warning, click "Yes" to continue, or resize your VM window first.

---

## Step 01: Development Environment

### 1.1 Login

1. **Login Screen:**
   - Click the green **"LOGIN"** button
   - This opens your browser for authentication
   - Sign in to your NVIDIA Developer account (create one if needed - it's free)
   - After login, SDK Manager will automatically proceed

### 1.2 Product Category

- **Select:** ✅ **"Jetson"** (should be selected by default)
- **Optional:** You can also select "Data Science" if you plan to use it, but it's not required

### 1.3 System Configuration

**Host Machine:**
- Should auto-detect: **"Ubuntu 22.04 - x86_64"** ✅
- If correct, you'll see a green checkmark
- No action needed if correct

**Target Hardware:**
- **IMPORTANT:** Click on **"Jetson Orin Nano modules"** (the row with the green checkmark)
- This will expand or show a dropdown with specific variants
- **Select:** **"Jetson Orin Nano [8GB developer kit version]"** or **"Jetson Orin Nano Developer Kit [8GB]"**
- ⚠️ **CRITICAL:** Make sure you select the **8GB version**, NOT the 4GB version!
- **How to find it:**
  - After clicking "Jetson Orin Nano modules", look for sub-options
  - Look for text that says "8GB" or "8 GB"
  - Avoid any option that says "4GB" or "4 GB"
  - If you don't see variants, try double-clicking or looking for a dropdown arrow
- ⚠️ **Warning:** You'll see a red warning "Could not detect a board" - this is **NORMAL** until you connect your Jetson
- The board will be detected when you connect it in recovery mode later
- **Note:** If you accidentally selected 4GB, you'll need to go back and change it, or restart SDK Manager
- **See also:** `SELECTING_8GB_VARIANT.md` for detailed instructions

### 1.4 SDK Version

- **Select:** **"JetPack 6.2.1"** or **"JetPack 6.2"** (recommended)
- Or choose the latest available version
- You can click "See what's new" to view release notes

### 1.5 Additional SDKs (Optional)

- **DeepStream:** Leave unchecked (optional, can add later)
- **GXF Runtime:** Leave unchecked (optional, can add later)
- **Recommendation:** Leave both unchecked for now to keep installation smaller

### 1.6 Continue

- Click **"CONTINUE > TO STEP 02"** button (green, bottom right)

---

## Step 02: Details and License

### 2.1 Review Target Components

**Jetson Runtime Components:**
- ✅ **"Additional Setups"** (4.0 MB) - Keep checked
- ✅ **"CUDA Runtime"** (2,199 MB) - Keep checked (required)
- ✅ **"CUDA X-AI Runtime"** (1,197 MB) - Keep checked (for AI workloads)
- ✅ **"Computer Vision Runtime"** (45.2 MB) - Keep checked (for vision tasks)
- ✅ **"NVIDIA"** - Keep checked
- ✅ **"Multi"** - Keep checked

**Other Components:**
- Most other components are optional
- You can leave them unchecked unless you specifically need them
- Scroll down to see all available components

### 2.2 Disk Space Requirements

- **System requires:** Up to 42GB (host) and 4GB (target) of available disk space
- Make sure you have enough space on your VM

### 2.3 Folder Paths

**Download folder:**
- Default: `/home/fadwa/Downloads/nvidia/sdkm_downloads`
- Shows: "(11GB required)"
- You can click "change" to select a different location if needed

**SDKs install folder:**
- Default: `/home/fadwa/nvidia/nvidia_sdk`
- Shows: "(31GB required)"
- You can click "change" to select a different location if needed

**Recommendation:** Use the default paths unless you have a specific reason to change them

### 2.4 License Agreement

- ✅ **Check:** "I accept the terms and conditions of the license agreements."
- You must accept to continue

### 2.5 Download Option

- **"Download now. Install later."** - Leave **unchecked** (we'll download and install together)
- If you check this, it will only download files without installing

### 2.6 Password Prompt

- When you click "CONTINUE > TO STEP 03", you'll be prompted for your Ubuntu password
- **Enter your password** (characters won't show - this is normal for security)
- Click **"OK"** or press Enter

### 2.7 Continue

- Click **"CONTINUE > TO STEP 03"** button (green, bottom right)

---

## Step 03: Setup Process

### 3.1 Overview

This step shows the installation progress. You'll see:

1. **Downloading components** (can take 30-60 minutes depending on internet speed)
2. **Installing components** (can take 20-40 minutes)
3. **Preparing flash** (when you connect your Jetson)

### 3.2 What Happens

- SDK Manager will download all selected components
- It will install them on your host machine
- Progress bars will show download/install status
- You can see detailed logs if needed

### 3.3 Flash Configuration Screen

**After download/installation completes, you'll see a screen titled:**
**"SDK Manager is about to flash your Jetson Orin Nano module"**

This screen has several important settings:

#### 3.3.1 Recovery Mode Setup (IMPORTANT!)

**For FIRST-TIME FLASHING (your situation):**
- **Change dropdown from "Automatic Setup" to "Manual Setup"**
- **Why:** Automatic Setup is only for devices that are already flashed and running
- **Manual Setup** is required for first-time flashing

**Steps:**
1. Click the **"Recovery mode setup:"** dropdown
2. **IMPORTANT:** Select **"Manual Setup - Jetson Orin Nano [8GB developer kit version]"**
   - ⚠️ **Make sure you select the 8GB version** (not 4GB)
   - If you see "4GB" in the dropdown, that's wrong - you need "8GB"

**⚠️ CRITICAL: If dropdown shows "4GB" or defaults to 4GB every time you click Refresh:**
- **Problem:** Step 01 selected 4GB instead of 8GB
- **Solution:** 
  1. Click **"Back"** button (bottom left) to go back to Step 01
  2. In Step 01 → System Configuration → Target Hardware
  3. Change to **"Jetson Orin Nano [8GB developer kit version]"**
  4. Continue through Step 02 and Step 03 (will be quick, no re-download)
  5. Return to Flash Configuration - should now show "8GB"
- **See:** `FIX_4GB_DEFAULT_ISSUE.md` for detailed step-by-step instructions

3. This will change the instructions shown below
4. The screen will update to show Manual Setup instructions (with jumper steps)

#### 3.3.2 Manual Setup Instructions (After Selecting Manual)

Once you select "Manual Setup", SDK Manager will show specific instructions. Follow these steps **in order**:

**Step 1: Power Off and Disconnect**
- Ensure the Jetson is **powered off**
- **Disconnect the power adapter** from the Power Jack [J16]

**Step 2: Verify Storage Device**
- Make sure your **SD card** (or other storage device) is **connected/inserted** into the Jetson
- For SD card: Insert it into the SD card slot on the Jetson

**Step 3: Place Jumper for Force Recovery Mode**
- Locate the **Button Header [J14]** on your Jetson board
- **Using your jumper wires from your purchase:**
  - ✅ **You have "Assorted Jumper Wires (M-F, F-F, M-M)" in your Phase 1 cart** - these will work!
  - For this step, you can use:
    - **Option A (Recommended):** A short **M-M (Male-to-Male)** jumper wire
      - Use one end to connect to pin 9, the other end to pin 10
      - You can cut it shorter if needed, or just use a short piece
    - **Option B:** A **F-F (Female-to-Female)** jumper wire
      - You'll need two small pieces of wire or pins to connect to the header
      - Less ideal, but will work if that's all you have
    - **Option C:** A dedicated **jumper cap** (small plastic cap that fits over two pins)
      - These are often included with development boards
      - If you have one, this is the cleanest option
- Place the jumper **across pins 9 and 10** of the Button Header [J14]
- ⚠️ **Important:** 
  - Make sure the jumper is securely placed on the correct pins
  - The pins are numbered - count carefully to ensure you're on pins 9 and 10
  - The jumper should make a solid electrical connection between the two pins
  - If using M-M wire, make sure both ends are firmly connected
- This puts the Jetson into Force Recovery Mode

**💡 Tip:** If you don't have jumper wires yet, you can also use:
- A small piece of wire (even a paperclip in a pinch, but jumper wire is safer)
- A small piece of solid-core wire stripped at both ends
- Any conductive material that can safely bridge the two pins

**Step 4: Connect USB-C Cable**
- Connect your **USB Type-C cable** from your Mac to the **USB Type-C connector** on the Jetson
- This is the connector on the front/edge of the board

**Step 5: Connect Power Adapter**
- Connect the **power adapter** to the **Power Jack [J16]**
- ⚠️ **The device will automatically power on in Force Recovery Mode** (this is expected!)
- You don't need to press any power button

**Step 6: Remove Jumper**
- **Remove the jumper** from pins 9 and 10 of the Button Header [J14]
- Keep the jumper safe - you may need it again

**Step 7: Connect to VM**
- **In UTM:**
  1. Look for the **USB icon** in the toolbar (or right-click the VM window)
  2. You should see a menu with USB devices
  3. Look for **"USB Device [05ac:1460] at 1-3"** or similar (this is your Jetson in recovery mode)
  4. **Click on the device** to open its submenu
  5. **Click "Connect..."** to attach it to the VM
  6. The device should now show as connected/attached
- **Alternative method:**
  - VM Settings → USB → Enable USB sharing
  - The device should appear in the list
  - Select it and click "Connect"
- The Jetson should appear as a USB device (may show as "APX", "NVIDIA", "L4T", or "USB Device [05ac:1460]" in the USB list)
- ⚠️ **Important:** Keep the USB device connected to the VM throughout the flashing process

**Step 8: Refresh Detection**

**Before clicking Refresh, verify these steps:**

1. **Check "Selected Device" at top of dialog:**
   - Look at the very top of the SDK Manager window
   - There should be a "Selected Device" dropdown or display
   - ⚠️ **It MUST say "Jetson Orin Nano [8GB developer kit version]"** (not 4GB!)
   - If it shows "4GB", you need to go back to Step 01 and select the correct model
   - The orange note says: "The 'Selected Device' options at the very top of this dialog may change after setting a board to recovery mode. Please make sure the correct board has been selected and then continue flashing."

2. **Verify Jetson is in recovery mode:**
   - Make sure you followed Steps 1-7 above (jumper method)
   - Jetson should be powered on (auto-powered when you connected power adapter)
   - USB-C cable should be connected
   - Jumper should be removed (after power-on)

3. **Verify USB device is connected to VM:**
   - In VM terminal, run: `lsusb | grep 05ac:1460`
   - Should show: `Bus 005 Device 004: ID 05ac:1460 Apple, Inc.`
   - If you don't see it, reconnect USB device in UTM

4. **Click Refresh:**
   - In SDK Manager, click **"Refresh"** next to the "Could not detect a board" warning
   - Wait 5-10 seconds for SDK Manager to scan for devices
   - SDK Manager should now detect: "Jetson Orin Nano detected"
   - The red warning should disappear
   - You'll see the board information

5. **If it still doesn't detect OR if it defaults to 4GB:**
   - **⚠️ Check "Recovery mode setup" dropdown** - if it shows "4GB", you need to fix Step 01
   - **Check Selected Device at top** - this is often the issue!
   - **If dropdown shows "4GB":** Click "Back" → Go to Step 01 → Change to "8GB" → Continue
   - Verify in VM terminal: `lsusb` should show `ID 05ac:1460`
   - Make sure Jetson is actually in recovery mode (followed all jumper steps)
   - Try disconnecting and reconnecting USB device in UTM
   - Wait 5 seconds, then click Refresh again
   - Sometimes you need to click Refresh multiple times

**⚠️ Important Note:**
- The "Selected Device" options at the top of the dialog may change after setting the board to recovery mode
- **Make sure the correct board is selected** before continuing
- If multiple devices appear, select "Jetson Orin Nano [8GB developer kit version]"

#### 3.3.3 Storage Device Selection

- **Storage Device dropdown:** Should be set to **"SD Card"** ✅
- This is correct for most setups
- **Note:** A minimum of 64GB space is recommended on the SD card

#### 3.3.4 OEM Configuration (Username and Password Setup)

**For Manual Setup (first-time flashing):**

You'll see these fields in the "OEM Configuration" section:

1. **Pre-Config dropdown:**
   - Leave as **"Pre-Config"** (default)
   - This sets up a default configuration

2. **New Username:**
   - **Enter a username** for your Jetson (e.g., `jetson`, `admin`, or your preferred name)
   - This will be the login username for the Jetson after flashing
   - **Example:** `jetson` or `fadwa`

3. **New Password:**
   - **Enter a password** for your Jetson
   - This will be the login password for the Jetson after flashing
   - Choose a secure password you'll remember
   - **Example:** `jetson123` or your preferred password

**⚠️ Important:**
- These credentials will be used to log into your Jetson after flashing
- Write them down or remember them - you'll need them later!
- The username and password are required for Manual Setup

**For Automatic Setup (if device already flashed):**
- You would use existing credentials instead
- But for first-time flashing, use "New Username" and "New Password" fields

#### 3.3.6 Enable Flash Button

**The "Flash" button will be DISABLED (grayed out) until:**
- ✅ You've selected "Manual Setup"
- ✅ Jetson is detected (after connecting in recovery mode and clicking Refresh)
- ✅ **New Username** field is filled
- ✅ **New Password** field is filled
- ✅ Storage Device is selected (should be "SD Card")

**Once the board is detected:**
- The "Flash" button will become **enabled (green)**
- Click **"Flash"** to begin the flashing process
- **DO NOT click "Skip"** - that will skip the flashing step

#### 3.3.7 Important Notes

⚠️ **Firewall Warning:**
- You may need to turn off your firewall or allow NFS
- This is usually not needed for Manual Setup, but keep in mind

⚠️ **Boot Order:**
- After flashing, you may need to manually change the device boot order
- This is usually automatic, but be aware

⚠️ **Don't Disconnect:**
- Once flashing starts, **DO NOT disconnect** the USB cable
- The process can take 20-40 minutes
- Wait for completion message

### 3.4 Wait for Completion

- **Don't disconnect** during download/installation
- This process can take 1-2 hours total
- You can monitor progress in the SDK Manager window
- **Current progress:** You can see the installation percentage (e.g., "Installing: 73.26%")

**⚠️ Important: If you selected wrong target hardware (4GB vs 8GB):**
- **You DON'T need to redo the download/installation!**
- The JetPack components are mostly the same for 4GB and 8GB models
- The main difference is in the flashing configuration, not the downloaded files
- **What to do:**
  1. Let the current installation finish (don't cancel it)
  2. When you reach the Flash Configuration screen, check the "Selected Device" at the top
  3. If it shows "4GB" and you need "8GB":
     - You can try clicking "Back" to go to Step 01 and change it
     - OR restart SDK Manager after installation completes
     - The downloaded files will still be there, so you won't need to re-download
     - You'll just need to go through the configuration steps again (which is quick)

---

## Step 04: Summary Finalization

### 4.1 Review Summary

- Review what was installed
- Check that all components completed successfully
- Verify Jetson was flashed successfully

### 4.2 Next Steps

- SDK Manager will show next steps
- You may be prompted to set up the Jetson (username, password, etc.)
- Follow any on-screen instructions

### 4.3 Complete

- Click **"Finish"** or **"Close"** when done
- Your Jetson should now be flashed with JetPack!

---

## Troubleshooting

### Password Field Not Working

1. Click inside the password field first
2. Click the VM window to ensure it has focus
3. Try typing (characters won't show - this is normal)
4. Press Enter or click OK

### Jetson Not Detected / Flash Button Disabled

**If you see "Could not detect a board" and Flash button is grayed out:**

1. **Make sure you selected "Manual Setup"** (not Automatic Setup)
   - Click the "Recovery mode setup:" dropdown
   - Select "Manual Setup - Jetson Orin Nano [8GB developer kit version]"

2. **Put Jetson in recovery mode correctly (Manual Setup method):**
   - Power off Jetson and disconnect power adapter
   - Verify SD card is inserted
   - Place jumper across pins 9 and 10 of Button Header [J14]
   - Connect USB-C cable to Jetson
   - Connect power adapter (device will auto-power on)
   - Remove jumper from pins 9 and 10

3. **Connect to VM (CRITICAL STEP):**
   - **In UTM:** Right-click the VM window or click USB icon in toolbar
   - Look for **"USB Device [05ac:1460] at 1-3"** or similar (this is your Jetson)
   - **Click on the device** → **Click "Connect..."** in the submenu
   - The device must show as **connected/attached** to the VM
   - ⚠️ **The device must stay connected throughout flashing**

4. **Click "Refresh"** next to the warning message in SDK Manager

5. **Verify connection in VM:**
   - Open terminal in VM
   - Run: `lsusb` (should show NVIDIA or Apple device)
   - You should see something like: 
     - `Bus 005 Device 004: ID 05ac:1460 Apple, Inc. USB-C Digital AV Multiport Adapter` (Jetson in recovery mode)
     - or `Bus 001 Device 003: ID 0955:XXXX NVIDIA Corp.`
   - ✅ **If you see `ID 05ac:1460`, your Jetson is connected!** (The "USB-C Digital AV Multiport Adapter" label is normal - it's your Jetson)
   - If you see the device, the connection is working

6. **If still not detected, try disconnecting and reconnecting:**
   - In UTM: Disconnect the USB device from VM
   - Put Jetson back in recovery mode (repeat steps 2)
   - In UTM: Reconnect the USB device (click "Connect...")
   - Click Refresh in SDK Manager again

7. **Check USB connection:**
   - Make sure you're using the **front USB Type-C connector** on Jetson
   - Try a different USB-C cable if available
   - Make sure the cable supports data (not just charging)
   - Try a different USB port on your Mac

### Error: "libusb_open: No such device (it may have been disconnected) [-4]"

**This error means the USB device is not connected to the VM or was disconnected:**

1. **Check USB device connection in UTM:**
   - Right-click VM window or click USB icon
   - Look for your Jetson device (e.g., "USB Device [05ac:1460]")
   - **Make sure it shows as "Connected" or has a checkmark**
   - If not connected, click on it → Click "Connect..."

2. **Reconnect the device:**
   - In UTM: Disconnect the USB device (if connected)
   - Wait 2-3 seconds
   - Reconnect it: Click on device → "Connect..."
   - Wait for it to show as connected

3. **Verify in VM terminal:**
   ```bash
   # In Ubuntu VM terminal
   lsusb
   # Should show your Jetson device
   # ✅ Good: Bus 005 Device 004: ID 05ac:1460 Apple, Inc. USB-C Digital AV Multiport Adapter
   # (The "USB-C Digital AV Multiport Adapter" label is normal - it's your Jetson in recovery mode)
   ```

4. **If device keeps disconnecting:**
   - Make sure USB cable is firmly connected to both Mac and Jetson
   - Try a different USB port on your Mac
   - Try a different USB-C cable
   - Make sure Jetson stays in recovery mode (don't remove power)

5. **Restart the connection process:**
   - Disconnect USB from VM
   - Power off Jetson (disconnect power adapter)
   - Put Jetson back in recovery mode (jumper method)
   - Connect USB-C to Mac
   - Connect power adapter
   - Remove jumper
   - In UTM: Connect USB device to VM
   - Click Refresh in SDK Manager

6. **Check UTM USB settings:**
   - VM Settings → USB
   - Make sure "Enable USB sharing" is checked
   - Try enabling "USB 3.0 support" if available

### Error: "The connected Jetson device is not ready to flash" / "Device is not ready to flash"

**This error means SDK Manager detected your Jetson, but the USB connection is not optimal for flashing. This is a common issue with VMs.**

⚠️ **If you've tried all solutions below and it still doesn't work, see: [ALTERNATIVE_FLASHING_METHODS.md](ALTERNATIVE_FLASHING_METHODS.md) for alternative approaches (using Windows/Linux computer, command-line tools, etc.).**

#### Solution 1: Switch to USB 2.0 (Most Common Fix)

**The issue is often that USB 3.0 is too fast/unstable for flashing. Try USB 2.0:**

1. **Change UTM USB Settings:**
   - Stop the VM (if running)
   - VM Settings → USB
   - Change **"USB Support"** from **"USB 3.0 (XHCI)"** to **"USB 2.0 (EHCI)"**
   - Click **"Save"**
   - Restart the VM

2. **Reconnect Jetson:**
   - Disconnect USB device from VM (if connected)
   - Power off Jetson (disconnect power adapter)
   - Put Jetson back in recovery mode:
     - Place jumper on pins 9-10
     - Connect USB-C cable to Jetson
     - Connect power adapter (auto-powers on)
     - Remove jumper
   - In UTM: Connect USB device to VM
   - Wait 5 seconds for device to initialize

3. **In SDK Manager:**
   - Click **"Refresh"** next to the detection warning
   - Wait 10 seconds
   - Try clicking **"Flash"** again

#### Solution 2: Use a Different USB Port/Cable

1. **Try a different USB port on your Mac:**
   - If using a USB hub, try connecting directly to Mac
   - Try a different USB-C port
   - Avoid USB hubs or adapters if possible

2. **Try a different USB-C cable:**
   - Use a high-quality USB-C cable (not just a charging cable)
   - Make sure it supports data transfer
   - Try a shorter cable if possible (shorter = more stable)

3. **Reconnect everything:**
   - Disconnect USB from VM
   - Unplug USB-C from Mac
   - Power off Jetson
   - Wait 10 seconds
   - Put Jetson back in recovery mode (jumper method)
   - Connect USB-C to a different Mac port
   - Connect power adapter
   - Remove jumper
   - In UTM: Connect USB device to VM
   - Wait 5 seconds
   - Click Refresh in SDK Manager

#### Solution 3: Wait Longer for Device Initialization

**Sometimes the device needs more time to fully initialize:**

1. **After connecting USB device to VM:**
   - Wait **15-20 seconds** (not just 5 seconds)
   - Don't click Refresh immediately
   - Let the device fully initialize

2. **Verify device is stable:**
   ```bash
   # In VM terminal
   lsusb | grep 05ac:1460
   # Run this command multiple times over 10 seconds
   # The device should appear consistently (not disappear/reappear)
   ```

3. **If device keeps appearing/disappearing:**
   - This indicates an unstable connection
   - Try Solution 1 (USB 2.0) or Solution 2 (different cable/port)

#### Solution 4: Check USB Device Connection Stability

1. **Monitor USB device in VM:**
   ```bash
   # In VM terminal, run this command repeatedly:
   watch -n 1 'lsusb | grep 05ac:1460'
   # Press Ctrl+C to stop
   ```
   - The device should appear **consistently** (not flicker)
   - If it disappears/reappears, the connection is unstable

2. **If unstable:**
   - Try USB 2.0 (Solution 1)
   - Try different cable/port (Solution 2)
   - Make sure USB cable is firmly connected
   - Avoid moving the cable during flashing

#### Solution 5: Restart Entire Process with USB 2.0

**Complete restart with USB 2.0 settings:**

1. **Close SDK Manager completely**

2. **Change UTM USB to 2.0:**
   - VM Settings → USB
   - Change to "USB 2.0 (EHCI)"
   - Save

3. **Restart VM**

4. **Put Jetson in recovery mode:**
   - Power off (disconnect power)
   - Insert SD card
   - Place jumper on pins 9-10
   - Connect USB-C to Mac (use a different port if possible)
   - Connect power adapter
   - Remove jumper

5. **Connect to VM:**
   - In UTM: Connect USB device to VM
   - Wait 15 seconds for initialization

6. **Verify in VM:**
   ```bash
   lsusb | grep 05ac:1460
   # Should show: Bus XXX Device XXX: ID 05ac:1460 Apple, Inc.
   ```

7. **Restart SDK Manager:**
   - Launch SDK Manager
   - Go through Step 01-03 again
   - On Step 04 (Flash Configuration):
     - Select "Manual Setup - Jetson Orin Nano [8GB developer kit version]"
     - Click Refresh
     - Wait 10 seconds
     - Enter username/password
     - Click Flash

#### Solution 6: Try Different VM USB Passthrough Method

**If using UTM toolbar method, try the settings method:**

1. **In UTM:**
   - VM Settings → USB
   - Make sure "Share USB devices from host" is checked
   - Look for your Jetson device in the list
   - Select it and make sure it's checked/enabled
   - Save settings

2. **Or try disconnecting/reconnecting:**
   - Right-click VM window → USB devices
   - Disconnect Jetson device (if connected)
   - Wait 5 seconds
   - Connect Jetson device again
   - Wait 10 seconds

#### Solution 7: USB Device Becomes Greyed Out After Detection (ID 0955:7523)

**If you see `ID 0955:7523 NVIDIA Corp. APX` in `lsusb` but USB device becomes greyed out in UTM when you try to connect it:**

**This means the Jetson is detected but the USB connection becomes unstable when passed through to the VM.**

1. **Connect APX Device BEFORE SDK Manager Detection:**
   - ⚠️ **CRITICAL:** Connect the APX device to the VM **BEFORE** SDK Manager tries to detect it
   - **Sequence:**
     - Power off Jetson (disconnect power adapter)
     - Place jumper on pins 9-10
     - Connect USB-C cable to Jetson
     - Connect power adapter (Jetson auto-powers on in recovery mode)
     - **Wait 10 seconds** for Jetson to fully enter recovery mode
     - **In UTM:** Right-click VM window → USB devices → Click on "APX (0:1)" → Click "Connect..."
     - **Wait 15 seconds** for device to stabilize in VM
     - **Verify connection:** Run `lsusb | grep 0955:7523` (should show device)
     - **Now** open/refresh SDK Manager
     - SDK Manager should detect it without the device greying out

2. **If Device Greys Out When Connecting:**
   - **Try this alternative method:**
     - Power off Jetson
     - **In UTM:** Make sure APX device is **NOT** connected yet
     - Place jumper on pins 9-10
     - Connect USB-C cable
     - Connect power adapter
     - **Wait 20 seconds** (longer wait - let Jetson fully stabilize)
     - **In UTM:** Right-click → USB devices → "APX (0:1)" → "Connect..."
     - **If it greys out immediately:** Disconnect it, wait 5 seconds, try connecting again
     - **Keep trying** - sometimes it takes 2-3 attempts

2. **Verify Jetson Stays in Recovery Mode:**
   ```bash
   # In VM terminal, monitor the device:
   watch -n 1 'lsusb | grep -E "0955:7523|05ac:1460"'
   # Should consistently show ID 0955:7523 (NVIDIA APX)
   # If it disappears or changes, Jetson is leaving recovery mode
   ```

3. **If Device Becomes Greyed Out:**
   - **In UTM:** The USB device will show as greyed out (disconnected)
   - **Don't panic** - this is the issue we're fixing
   - **Reconnect the USB device:**
     - Right-click VM window → USB devices
     - Look for the Jetson device (may show as "NVIDIA" or "APX" or device ID)
     - Click on it → Click "Connect..." or "Reconnect"
     - Wait 10 seconds

4. **Alternative: Keep Jumper In Place Longer:**
   - Some Jetson boards need the jumper to stay in place during initial detection
   - Try this sequence:
     - Power off Jetson
     - Place jumper on pins 9-10
     - Connect USB-C cable
     - Connect power adapter
     - **Keep jumper in place** (don't remove yet)
     - In UTM: Connect USB device to VM
     - Wait 10 seconds
     - In SDK Manager: Click Refresh
     - Wait for SDK Manager to detect Jetson
     - **Only after SDK Manager detects it:** Remove jumper
     - **Immediately click Flash** (don't wait too long)

5. **If USB Device Keeps Disconnecting:**
   - **Try a different USB-C cable** (high-quality, data-capable)
   - **Try a different USB port** on your Mac (direct connection, not hub)
   - **Make sure USB 2.0 is selected** in UTM (not USB 3.0)
   - **Check USB cable connection** - make sure it's firmly connected on both ends

6. **Timing is Critical:**
   - Connect USB device to VM
   - Wait 15 seconds for device to initialize
   - Click Refresh in SDK Manager
   - Wait for detection (should show "Jetson Orin Nano detected")
   - **Immediately enter username/password and click Flash**
   - Don't wait too long between detection and flashing

7. **If Still Failing - Try Manual Reconnection:**
   - When SDK Manager detects Jetson, **before clicking Flash:**
     - In UTM: Disconnect USB device (right-click → Disconnect)
     - Wait 3 seconds
     - Reconnect USB device (right-click → Connect...)
     - Wait 10 seconds
     - Verify in terminal: `lsusb | grep 0955:7523` (should show device)
     - **Now click Flash in SDK Manager**

#### Quick Checklist for "Device Not Ready to Flash"

- [ ] Changed UTM USB to USB 2.0 (EHCI) instead of USB 3.0
- [ ] Tried a different USB port on Mac
- [ ] Tried a different USB-C cable
- [ ] Waited 15-20 seconds after connecting USB device to VM
- [ ] Verified device appears consistently in `lsusb` (doesn't flicker)
- [ ] Restarted entire process (SDK Manager + VM + Jetson recovery mode)
- [ ] Made sure USB cable is firmly connected (not loose)
- [ ] Avoided USB hubs/adapters (connected directly to Mac)

**Most common fix: Switch to USB 2.0 in UTM settings!** USB 3.0 can be too fast/unstable for Jetson flashing in VMs.

### Device Connected but SDK Manager Still Shows "Could not detect a board"

**If `lsusb` shows your device (ID 05ac:1460) but SDK Manager doesn't detect it:**

1. **Check "Selected Device" at Top of Dialog (MOST COMMON ISSUE!):**
   - ⚠️ **CRITICAL:** Look at the **very top** of the SDK Manager window
   - There should be a "Selected Device" dropdown or display
   - It MUST say **"Jetson Orin Nano [8GB developer kit version]"**
   - **NOT "4GB"** - if it shows 4GB, that's the problem!
   - The orange note in SDK Manager says: "The 'Selected Device' options at the very top of this dialog may change after setting a board to recovery mode. Please make sure the correct board has been selected and then continue flashing."
   - **📖 For detailed instructions on how to check and fix this, see: [HOW_TO_CHECK_SELECTED_DEVICE.md](HOW_TO_CHECK_SELECTED_DEVICE.md)**
   - **If it shows 4GB:** 
     - **Option 1:** Click "Back" button → Go to Step 01 → System Configuration → Target Hardware → Select "Jetson Orin Nano [8GB developer kit version]"
     - **Option 2 (Recommended):** Close SDK Manager → Restart → In Step 01, select "8GB" version → Continue through steps

2. **Check Recovery Mode Selection:**
   - Make sure you selected **"Manual Setup - Jetson Orin Nano [8GB developer kit version]"**
   - **NOT "Automatic Setup"** (that's for already-flashed devices)
   - The dropdown should say "8GB developer kit version"

3. **Verify Jetson is Actually in Recovery Mode:**
   - Did you follow all the jumper steps?
   - Power off → Place jumper on pins 9-10 → Connect USB-C → Connect power → Remove jumper
   - Jetson should be powered on (auto-powered when you connected power)
   - USB-C cable should be connected to Mac

4. **Verify Device in VM:**
   ```bash
   # In VM terminal
   lsusb | grep -i "05ac:1460"
   # Should show: Bus 005 Device 004: ID 05ac:1460 Apple, Inc.
   ```
   - If you see this, the device IS connected correctly to the VM

5. **Click Refresh Multiple Times:**
   - Click the **"Refresh"** link next to "Could not detect a board"
   - Wait 5-10 seconds (SDK Manager needs time to scan)
   - If it doesn't detect, click Refresh again
   - Sometimes it takes 2-3 refresh attempts

6. **Try Reconnecting USB Device:**
   - In UTM: Disconnect USB device (right-click → Disconnect)
   - Wait 5 seconds
   - Reconnect USB device (right-click → Connect...)
   - Wait 3 seconds
   - In SDK Manager: Click Refresh again

7. **Verify USB Device is Still Connected:**
   ```bash
   # In VM terminal - check if device is still there
   lsusb
   # Should still show ID 05ac:1460
   ```

8. **If Still Not Working:**
   - Restart the entire process:
     - Close SDK Manager
     - Disconnect USB from VM
     - Power off Jetson (disconnect power)
     - Put Jetson back in recovery mode (jumper method)
     - Connect USB-C to Mac
     - Connect power adapter
     - Remove jumper
     - In UTM: Connect USB device to VM
     - Verify with `lsusb`
     - Reopen SDK Manager
     - Make sure "Selected Device" at top shows 8GB
     - Select "Manual Setup - 8GB"
     - Click Refresh

### Download/Install Fails

1. Check internet connection
2. Make sure you have enough disk space (42GB+)
3. Check logs in SDK Manager for specific errors
4. Try again - sometimes network issues cause temporary failures

### Installation Takes Too Long

- This is normal! Downloading and installing can take 1-2 hours
- Be patient and don't interrupt the process
- Make sure your Mac doesn't go to sleep

---

## Quick Reference Checklist

- [ ] Launched SDK Manager
- [ ] Logged in to NVIDIA Developer account
- [ ] Selected "Jetson" as product category
- [ ] Verified Host Machine: Ubuntu 22.04 - x86_64
- [ ] Selected Target Hardware: Jetson Orin Nano
- [ ] Selected SDK Version: JetPack 6.2.1 (or latest)
- [ ] Left Additional SDKs unchecked (optional)
- [ ] Continued to Step 02
- [ ] Reviewed and accepted target components
- [ ] Accepted license agreement
- [ ] Entered password when prompted
- [ ] Continued to Step 03
- [ ] Waited for download/installation (1-2 hours)
- [ ] Reached Flash Configuration screen
- [ ] Changed "Recovery mode setup" to "Manual Setup"
- [ ] Powered off Jetson and disconnected power adapter
- [ ] Verified SD card is inserted
- [ ] Placed jumper across pins 9 and 10 of Button Header [J14]
- [ ] Connected USB-C cable to Jetson
- [ ] Connected power adapter (device auto-powered on)
- [ ] Removed jumper from pins 9 and 10
- [ ] Connected Jetson USB device to VM (UTM)
- [ ] Clicked "Refresh" in SDK Manager
- [ ] Verified Jetson detected (warning disappeared)
- [ ] Entered "New Username" in OEM Configuration
- [ ] Entered "New Password" in OEM Configuration
- [ ] Verified Storage Device is set to "SD Card"
- [ ] Flash button became enabled
- [ ] Verified correct board is selected at top of dialog
- [ ] Clicked "Flash" to start flashing
- [ ] Waited for flashing to complete (20-40 minutes)
- [ ] Completed flashing process
- [ ] Finished Step 04

---

**This process takes time, but follow these steps carefully and you'll have your Jetson flashed successfully!**

