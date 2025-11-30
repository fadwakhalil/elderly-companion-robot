# How to Check and Fix "Selected Device" in SDK Manager

**Quick guide to verify and change the board model selection in SDK Manager.**

---

## Where to Find "Selected Device"

The "Selected Device" setting appears in **two places** in SDK Manager:

### Location 1: Step 01 - System Configuration (Initial Setup)

1. **In Step 01 (Development Environment):**
   - Look for the section labeled **"System Configuration"** or **"Target Hardware"**
   - There should be a dropdown or selection box
   - This is where you first select your Jetson model

2. **What to look for:**
   - Should say: **"Jetson Orin Nano [8GB developer kit version]"**
   - **NOT:** "Jetson Orin Nano [4GB developer kit version]"

### Location 2: Top of Flash Configuration Screen (Current Screen)

1. **At the very top of the Flash Configuration screen:**
   - Look **above** the green title "SDK Manager is about to flash your Jetson Orin Nano module"
   - There may be a dropdown or text display showing the selected device
   - This might be labeled as "Selected Device" or "Target Device"

2. **Visual location:**
   ```
   [SDK Manager Window]
   ┌─────────────────────────────────────────┐
   │ Selected Device: [Jetson Orin Nano...] │ ← LOOK HERE (at the very top)
   ├─────────────────────────────────────────┤
   │ SDK Manager is about to flash...        │
   │ Could not detect a board (refresh)      │
   │ ...                                     │
   └─────────────────────────────────────────┘
   ```

---

## How to Check Current Selection

### Method 1: Look at the Flash Configuration Screen

1. **On the current Flash Configuration screen:**
   - Look at the **very top** of the window (above the green title)
   - Check if there's a dropdown or text showing the device model
   - It should say **"8GB"** somewhere in the text

2. **Also check the "Recovery mode setup" dropdown:**
   - It should say: **"Manual Setup - Jetson Orin Nano [8GB developer kit...]"**
   - If it says "4GB" here, that's a problem

### Method 2: Go Back to Step 01

1. **Click the back button** (usually at bottom left) or restart SDK Manager
2. **Go to Step 01 → System Configuration**
3. **Look at "Target Hardware" dropdown:**
   - What does it currently show?
   - Should be: **"Jetson Orin Nano [8GB developer kit version]"**

---

## How to Fix If It Shows 4GB

### Option A: Change in Step 01 (If You Can Go Back)

1. **Click "Back" or restart SDK Manager**
2. **Go to Step 01 → System Configuration**
3. **Click on "Target Hardware" dropdown**
4. **Select:** **"Jetson Orin Nano [8GB developer kit version]"**
   - Look for options that say "8GB" or "8 GB"
   - Avoid any that say "4GB" or "4 GB"
5. **Continue through the steps again**

### Option B: Restart SDK Manager (Recommended)

1. **Close SDK Manager completely**
2. **Reopen SDK Manager:**
   ```bash
   sdkmanager
   ```
3. **Go through Step 01 again:**
   - Login
   - Select "Jetson" as product category
   - **In System Configuration → Target Hardware:**
     - **Select: "Jetson Orin Nano [8GB developer kit version]"**
     - Make absolutely sure it says **8GB**, not 4GB
   - Select JetPack version (6.2.1)
   - Continue to Step 02
   - Continue to Step 03
   - When you reach Flash Configuration screen:
     - Select "Manual Setup - Jetson Orin Nano [8GB developer kit version]"
     - Click Refresh

---

## How to Verify You Have the Right Selection

### Check These Locations:

1. **Step 01 - Target Hardware:**
   - ✅ Should say: "Jetson Orin Nano [8GB developer kit version]"
   - ❌ Should NOT say: "Jetson Orin Nano [4GB developer kit version]"

2. **Flash Configuration - Recovery Mode Setup:**
   - ✅ Should say: "Manual Setup - Jetson Orin Nano [8GB developer kit...]"
   - ❌ Should NOT say: "Manual Setup - Jetson Orin Nano [4GB developer kit...]"

3. **Flash Configuration - Top of Window:**
   - ✅ Should mention "8GB" somewhere
   - ❌ Should NOT mention "4GB"

---

## Quick Checklist

- [ ] Checked Step 01 → Target Hardware → Shows "8GB" (not 4GB)
- [ ] Checked Flash Configuration → Recovery Mode Setup → Shows "8GB" (not 4GB)
- [ ] Checked top of Flash Configuration screen → Shows "8GB" (not 4GB)
- [ ] If any show "4GB", restarted SDK Manager and selected "8GB" in Step 01
- [ ] Verified USB device is connected (lsusb shows ID 05ac:1460)
- [ ] Clicked "Refresh" in SDK Manager

---

## Still Not Working?

If you've verified all selections show "8GB" but SDK Manager still doesn't detect:

1. **Verify Jetson is in recovery mode:**
   - Followed all jumper steps?
   - Powered on?
   - USB-C connected?

2. **Verify USB connection:**
   ```bash
   lsusb | grep 05ac:1460
   ```
   Should show your device.

3. **Try clicking Refresh multiple times:**
   - Sometimes it takes 2-3 attempts

4. **Try reconnecting USB device in UTM:**
   - Disconnect → Wait 5 seconds → Reconnect → Click Refresh

---

**The key is making sure EVERYWHERE in SDK Manager shows "8GB", not "4GB"!**


