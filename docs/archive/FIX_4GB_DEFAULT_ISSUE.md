# Fix: SDK Manager Defaults to 4GB Every Time

**Problem:** Every time you click "Refresh" in Flash Configuration, it defaults to "Jetson Orin Nano 4GB" instead of 8GB.

**Root Cause:** The initial selection in Step 01 was set to 4GB, so SDK Manager remembers that setting.

---

## ⚠️ The Problem

You're seeing:
- ❌ "Recovery mode setup: Automatic Setup - Jetson Orin Nano 4GB"
- ❌ Every refresh defaults back to 4GB
- ❌ Can't change it to 8GB on this screen

**This is because Step 01 selected 4GB, and SDK Manager uses that as the default.**

---

## ✅ Solution: Go Back to Step 01 and Fix It

### Option 1: Use Back Button (Quickest)

1. **On the Flash Configuration screen:**
   - Look for a **"Back"** or **"< Back"** button (usually at bottom left)
   - Click it to go back to Step 03

2. **Keep clicking "Back" until you reach Step 01:**
   - Step 04 (Flash Configuration) → Step 03 → Step 02 → Step 01

3. **In Step 01 - System Configuration:**
   - Find **"Target Hardware"** section
   - Click on the current selection (probably shows "Jetson Orin Nano [4GB developer kit version]")
   - **Change it to:** "Jetson Orin Nano [8GB developer kit version]"
   - Make absolutely sure it says **"8GB"**, not "4GB"

4. **Continue through the steps:**
   - Step 02: Just review (should be quick, no re-download needed)
   - Step 03: Should skip (already installed)
   - Step 04: Now should show "8GB" in the dropdown

5. **On Flash Configuration screen:**
   - Change "Recovery mode setup" to **"Manual Setup - Jetson Orin Nano [8GB developer kit version]"**
   - Click "Refresh"
   - Should now detect correctly

---

### Option 2: Restart SDK Manager (If Back Button Doesn't Work)

1. **Close SDK Manager completely:**
   - Click the X button or close the window
   - Make sure it's fully closed (not just minimized)

2. **Reopen SDK Manager:**
   ```bash
   sdkmanager
   ```

3. **Go through Step 01 again:**
   - Login (if needed)
   - Select "Jetson" as product category
   - **In System Configuration → Target Hardware:**
     - **CRITICAL:** Click on "Jetson Orin Nano modules"
     - Expand to see variants
     - **Select: "Jetson Orin Nano [8GB developer kit version]"**
     - **DOUBLE-CHECK:** Make absolutely sure it says **"8GB"**, not "4GB"
   - Select JetPack version (6.2.1)
   - Continue to Step 02

4. **Step 02 - Details and License:**
   - Review components (should be quick)
   - SDK Manager will detect your existing downloads
   - Click "CONTINUE > TO STEP 03"

5. **Step 03 - Setup Process:**
   - Should be quick (won't re-download, just verify existing installation)
   - Click "CONTINUE > TO STEP 04" when done

6. **Step 04 - Flash Configuration:**
   - **Check "Recovery mode setup" dropdown:**
     - Should now show: **"Manual Setup - Jetson Orin Nano [8GB developer kit version]"**
     - If it still shows "4GB", you didn't select correctly in Step 01 - go back and fix it
   - **Change to "Manual Setup"** if it's on "Automatic Setup"
   - Click "Refresh"
   - Should now work correctly

---

## 🔍 How to Verify You Fixed It

**After going back to Step 01 and changing to 8GB, verify:**

1. **Step 01 - Target Hardware:**
   - ✅ Shows: "Jetson Orin Nano [8GB developer kit version]"
   - ❌ Does NOT show: "Jetson Orin Nano [4GB developer kit version]"

2. **Step 04 - Flash Configuration:**
   - ✅ "Recovery mode setup" shows: "Manual Setup - Jetson Orin Nano [8GB developer kit version]"
   - ❌ Does NOT show: "4GB" anywhere
   - ✅ When you click "Refresh", it stays on "8GB"

---

## 📝 Step-by-Step: Changing Recovery Mode Setup

**After fixing the 4GB/8GB issue, you also need to change to Manual Setup:**

1. **On Flash Configuration screen:**
   - Find the dropdown: **"Recovery mode setup:"**
   - Currently shows: "Automatic Setup - Jetson Orin Nano 4GB" (or 8GB after fix)

2. **Click the dropdown:**
   - Select: **"Manual Setup - Jetson Orin Nano [8GB developer kit version]"**
   - **NOT:** "Automatic Setup" (that's for already-flashed devices)

3. **Fill in the fields:**
   - **New Username:** Enter a username (e.g., `jetson`)
   - **New Password:** Enter a password (e.g., `jetson123`)
   - **Storage Device:** Should be "SD Card"

4. **Connect Jetson in recovery mode:**
   - Follow the jumper steps to put Jetson in recovery mode
   - Connect USB-C cable
   - Connect USB device to VM (in UTM: USB menu → select device)

5. **Click "Refresh":**
   - Should now detect the board
   - "Flash" button should become enabled

---

## ⚠️ Important Notes

**Why it keeps defaulting to 4GB:**
- SDK Manager remembers your Step 01 selection
- The Flash Configuration screen uses that selection as the default
- You can't permanently change it on the Flash Configuration screen
- **You MUST fix it in Step 01**

**Why you need Manual Setup:**
- "Automatic Setup" is for devices that are already flashed and running
- "Manual Setup" is for first-time flashing (your situation)
- Manual Setup requires putting Jetson in recovery mode

**Your downloads are safe:**
- Going back to Step 01 won't delete your downloads
- Step 03 will be quick (just verification, no re-download)
- You won't lose progress

---

## 🎯 Quick Checklist

- [ ] Clicked "Back" to go to Step 01 (or restarted SDK Manager)
- [ ] In Step 01, changed "Target Hardware" to "Jetson Orin Nano [8GB developer kit version]"
- [ ] Verified it says "8GB" (not "4GB")
- [ ] Continued through Step 02 and Step 03
- [ ] On Flash Configuration screen, "Recovery mode setup" now shows "8GB"
- [ ] Changed "Recovery mode setup" to "Manual Setup - Jetson Orin Nano [8GB developer kit version]"
- [ ] Clicked "Refresh" - stays on "8GB" (doesn't default back to 4GB)

---

## 🚨 If It Still Defaults to 4GB After Fixing

**If you've changed Step 01 to 8GB but Flash Configuration still shows 4GB:**

1. **Double-check Step 01:**
   - Make absolutely sure you selected "8GB" variant
   - Not just "Jetson Orin Nano modules" (that's the parent category)
   - Must be the specific "8GB developer kit version"

2. **Try restarting SDK Manager:**
   - Close completely
   - Reopen
   - Go through Step 01 again, carefully selecting 8GB

3. **Check if there's a cache:**
   - SDK Manager might cache the old selection
   - Restarting should clear it

---

**The key is: Fix it at the source (Step 01), not on the Flash Configuration screen!** 🎯

