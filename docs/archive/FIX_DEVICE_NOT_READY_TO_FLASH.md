# Fix: "Device is not ready to flash" Error

**Quick reference guide for fixing the "The connected Jetson device is not ready to flash" error in SDK Manager.**

---

## ⚡ Quick Fix (Try This First!)

**If you see `ID 0955:7523 NVIDIA Corp. APX` in `lsusb` but USB device becomes greyed out:**

**The Jetson is detected but USB connection drops when it tries to flash. Try this:**

1. **Keep jumper in place longer:**
   - Power off Jetson
   - Place jumper on pins 9-10
   - Connect USB-C cable
   - Connect power adapter
   - **Keep jumper in place** (don't remove yet)
   - In UTM: Connect USB device to VM
   - Wait 10 seconds
   - In SDK Manager: Click Refresh
   - **Only after SDK Manager detects it:** Remove jumper
   - **Immediately click Flash** (don't wait)

2. **If that doesn't work, switch to USB 2.0:**
   - Stop your VM
   - VM Settings → USB
   - Change "USB Support" from "USB 3.0 (XHCI)" to "USB 2.0 (EHCI)"
   - Save and restart VM
   - Reconnect Jetson in recovery mode
   - Try flashing again

---

## Why This Happens

This error occurs when:
- SDK Manager detects your Jetson in recovery mode ✅
- But the USB connection is not stable/optimal for flashing ❌

**Common causes:**
- USB 3.0 is too fast/unstable for flashing in VMs
- USB cable quality issues
- USB port issues
- Device not fully initialized
- Unstable USB passthrough in VM

---

## Step-by-Step Solutions

### Solution 1: Switch to USB 2.0 (90% of cases)

**This fixes the issue in most cases:**

1. **Stop the VM** (if running)

2. **Change USB Settings:**
   - Open VM Settings
   - Click **"USB"** in left sidebar
   - Change **"USB Support"** dropdown:
     - From: **"USB 3.0 (XHCI)"**
     - To: **"USB 2.0 (EHCI)"**
   - Click **"Save"**

3. **Restart VM**

4. **Reconnect Jetson:**
   - Disconnect USB device from VM (if connected)
   - Power off Jetson (disconnect power adapter)
   - Put Jetson in recovery mode:
     - Place jumper on pins 9-10
     - Connect USB-C cable
     - Connect power adapter
     - Remove jumper
   - In UTM: Connect USB device to VM
   - **Wait 15-20 seconds** (important!)

5. **In SDK Manager:**
   - Click **"Refresh"**
   - Wait 10 seconds
   - Click **"Flash"**

---

### Solution 2: Try Different USB Port/Cable

**If Solution 1 doesn't work:**

1. **Try a different USB port on your Mac:**
   - Avoid USB hubs
   - Try connecting directly to Mac
   - Try a different USB-C port

2. **Try a different USB-C cable:**
   - Use a high-quality cable
   - Make sure it supports data (not just charging)
   - Try a shorter cable

3. **Reconnect everything:**
   - Follow the reconnection steps from Solution 1
   - Use the new port/cable

---

### Solution 3: Wait Longer for Initialization

**Sometimes the device needs more time:**

1. **After connecting USB device to VM:**
   - Wait **15-20 seconds** (not just 5 seconds)
   - Don't click Refresh immediately

2. **Verify device is stable:**
   ```bash
   # In VM terminal
   lsusb | grep 05ac:1460
   # Run this multiple times - device should appear consistently
   ```

3. **If device flickers (appears/disappears):**
   - Connection is unstable
   - Try Solution 1 (USB 2.0) or Solution 2 (different cable)

---

### Solution 4: Complete Restart

**If nothing else works:**

1. **Close SDK Manager completely**

2. **Change UTM to USB 2.0:**
   - VM Settings → USB → USB 2.0 (EHCI)
   - Save

3. **Restart VM**

4. **Put Jetson in recovery mode:**
   - Power off
   - Insert SD card
   - Place jumper on pins 9-10
   - Connect USB-C to Mac (different port)
   - Connect power adapter
   - Remove jumper

5. **Connect to VM:**
   - In UTM: Connect USB device
   - **Wait 20 seconds**

6. **Verify:**
   ```bash
   lsusb | grep 05ac:1460
   # Should show: Bus XXX Device XXX: ID 05ac:1460
   ```

7. **Restart SDK Manager:**
   - Launch SDK Manager
   - Go through Step 01-03
   - On Step 04:
     - Select "Manual Setup - 8GB"
     - Click Refresh
     - Wait 10 seconds
     - Enter username/password
     - Click Flash

---

## Verification Steps

**Before clicking Flash, verify:**

1. **USB device is connected to VM:**
   ```bash
   lsusb | grep 05ac:1460
   # Should show your Jetson device
   ```

2. **Device appears consistently:**
   ```bash
   watch -n 1 'lsusb | grep 05ac:1460'
   # Device should stay visible (not flicker)
   # Press Ctrl+C to stop
   ```

3. **SDK Manager shows correct device:**
   - "Selected Device" at top should say "8GB"
   - Recovery mode setup should say "Manual Setup - 8GB"
   - No red warnings about detection

4. **USB settings:**
   - UTM USB Support: USB 2.0 (EHCI) ✅
   - USB Sharing: Enabled ✅

---

## Troubleshooting Checklist

- [ ] Changed UTM USB to USB 2.0 (EHCI)
- [ ] Tried different USB port on Mac
- [ ] Tried different USB-C cable
- [ ] Waited 15-20 seconds after connecting USB device
- [ ] Verified device appears consistently in `lsusb`
- [ ] Restarted entire process (SDK Manager + VM + Jetson)
- [ ] USB cable is firmly connected (not loose)
- [ ] Not using USB hub (connected directly to Mac)
- [ ] Jetson is in recovery mode (jumper method completed)
- [ ] SD card is inserted

---

### Solution 7: USB Device Becomes Greyed Out (ID 0955:7523)

**If you see `ID 0955:7523 NVIDIA Corp. APX` in `lsusb` but USB device becomes greyed out in UTM:**

**This means the Jetson is detected but the USB connection drops when it tries to flash.**

1. **Keep Jumper In Place Longer:**
   - Power off Jetson
   - Place jumper on pins 9-10
   - Connect USB-C cable
   - Connect power adapter
   - **Keep jumper in place** (don't remove yet)
   - In UTM: Connect USB device to VM
   - Wait 10 seconds
   - In SDK Manager: Click Refresh
   - Wait for SDK Manager to detect Jetson
   - **Only after detection:** Remove jumper
   - **Immediately click Flash** (don't wait too long)

2. **Monitor Device Stability:**
   ```bash
   # In VM terminal
   watch -n 1 'lsusb | grep -E "0955:7523|05ac:1460"'
   # Should consistently show ID 0955:7523
   # If it disappears, Jetson is leaving recovery mode
   ```

3. **If Device Becomes Greyed Out:**
   - In UTM: Reconnect USB device (right-click → Connect...)
   - Wait 10 seconds
   - Verify: `lsusb | grep 0955:7523`
   - Click Flash immediately

4. **Timing is Critical:**
   - Connect USB device to VM
   - Wait 15 seconds
   - Click Refresh in SDK Manager
   - Wait for detection
   - **Immediately enter username/password and click Flash**
   - Don't wait between detection and flashing

## Still Not Working?

**If none of the above solutions work:**

1. **Check UTM version:**
   - Make sure you're using the latest version of UTM
   - Update if needed

2. **Try VirtualBox instead:**
   - If on Intel Mac, try VirtualBox
   - See [VIRTUALBOX_SETUP.md](VIRTUALBOX_SETUP.md)

3. **Try on a different computer:**
   - Use a Windows/Linux computer with SDK Manager
   - Or use a friend's computer

4. **Check Jetson hardware:**
   - Make sure Jetson is not damaged
   - Try a different Jetson if available

---

## Why USB 2.0 Works Better

- **USB 3.0** is faster but can be unstable in VMs
- **USB 2.0** is slower but more stable for flashing
- Jetson flashing doesn't need USB 3.0 speed
- USB 2.0 provides more reliable connection in virtualized environments

---

**Most users fix this by switching to USB 2.0. Try that first!** 🚀

